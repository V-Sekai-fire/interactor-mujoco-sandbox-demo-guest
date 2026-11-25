extern "C" {
#include <ei.h>
}
#include "mujoco_host.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>

static int listen_on_any_port(int &port) {
	const int fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		return -1;
	}
	int on = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = 0;
	if (bind(fd, (sockaddr *)&addr, sizeof(addr)) < 0 || listen(fd, 5) < 0) {
		close(fd);
		return -1;
	}
	socklen_t len = sizeof(addr);
	getsockname(fd, (sockaddr *)&addr, &len);
	port = ntohs(addr.sin_port);
	return fd;
}

static void reply_error(ei_x_buff &out, const char *reason) {
	ei_x_encode_tuple_header(&out, 2);
	ei_x_encode_atom(&out, "error");
	ei_x_encode_atom(&out, reason);
}

static void reply_info(ei_x_buff &out, MujocoHost &host, const std::vector<std::string> &names) {
	ei_x_encode_tuple_header(&out, 4);
	ei_x_encode_atom(&out, "ok");
	ei_x_encode_long(&out, host.cached_nq());
	ei_x_encode_long(&out, host.cached_nbody());
	ei_x_encode_list_header(&out, (long)names.size());
	for (const std::string &n : names) {
		ei_x_encode_binary(&out, n.data(), (long)n.size());
	}
	ei_x_encode_empty_list(&out);
}

static void reply_fk(ei_x_buff &out, MujocoHost &host, const std::vector<std::uint8_t> &qpos) {
	const int nq = host.cached_nq();
	const int nbody = host.cached_nbody();
	if (qpos.size() != sizeof(double) * (std::size_t)nq) {
		reply_error(out, "bad_qpos_size");
		return;
	}
	std::vector<double> q((std::size_t)nq), bodies((std::size_t)nbody * 7);
	std::memcpy(q.data(), qpos.data(), qpos.size());
	if (!host.fk(q.data(), nq, bodies.data(), nbody)) {
		reply_error(out, "fk_failed");
		return;
	}
	ei_x_encode_tuple_header(&out, 2);
	ei_x_encode_atom(&out, "ok");
	ei_x_encode_binary(&out, bodies.data(), (long)(bodies.size() * sizeof(double)));
}

static bool serve(int fd, MujocoHost &host, const std::vector<std::string> &names) {
	ei_x_buff in;
	ei_x_new(&in);
	bool stop = false;
	for (;;) {
		erlang_msg msg;
		const int got = ei_xreceive_msg(fd, &msg, &in);
		if (got == ERL_TICK) {
			continue;
		}
		if (got == ERL_ERROR) {
			break;
		}
		if (msg.msgtype != ERL_REG_SEND && msg.msgtype != ERL_SEND) {
			continue;
		}
		int index = 0, version = 0, arity = 0;
		erlang_pid from;
		ei_x_buff out;
		ei_x_new_with_version(&out);
		if (ei_decode_version(in.buff, &index, &version) < 0 || ei_decode_tuple_header(in.buff, &index, &arity) < 0 || arity != 2 || ei_decode_pid(in.buff, &index, &from) < 0) {
			ei_x_free(&out);
			continue;
		}
		int type = 0, size = 0;
		ei_get_type(in.buff, &index, &type, &size);
		char atom[MAXATOMLEN];
		if (type == ERL_ATOM_EXT || type == ERL_SMALL_ATOM_EXT || type == ERL_ATOM_UTF8_EXT || type == ERL_SMALL_ATOM_UTF8_EXT) {
			ei_decode_atom(in.buff, &index, atom);
			if (std::strcmp(atom, "info") == 0) {
				reply_info(out, host, names);
			} else if (std::strcmp(atom, "stop") == 0) {
				ei_x_encode_atom(&out, "ok");
				stop = true;
			} else {
				reply_error(out, "unknown_request");
			}
		} else if (type == ERL_SMALL_TUPLE_EXT || type == ERL_LARGE_TUPLE_EXT) {
			int inner = 0;
			ei_decode_tuple_header(in.buff, &index, &inner);
			if (inner == 2 && ei_decode_atom(in.buff, &index, atom) == 0 && std::strcmp(atom, "fk") == 0) {
				int btype = 0, bsize = 0;
				ei_get_type(in.buff, &index, &btype, &bsize);
				if (btype == ERL_BINARY_EXT) {
					std::vector<std::uint8_t> qpos((std::size_t)bsize);
					long got_len = 0;
					ei_decode_binary(in.buff, &index, qpos.data(), &got_len);
					reply_fk(out, host, qpos);
				} else {
					reply_error(out, "fk_needs_a_binary");
				}
			} else {
				reply_error(out, "unknown_request");
			}
		} else {
			reply_error(out, "unknown_request");
		}
		ei_send(fd, &from, out.buff, out.index);
		ei_x_free(&out);
		in.index = 0;
		if (stop) {
			break;
		}
	}
	ei_x_free(&in);
	return stop;
}

int main(int argc, char **argv) {
	const char *cookie = std::getenv("MUJOCO_COOKIE");
	if (argc != 3 || cookie == nullptr || *cookie == 0) {
		std::fprintf(stderr, "usage: MUJOCO_COOKIE=<cookie> %s <mujoco ELF> <node name, for example mujoco>\n", argv[0]);
		return 2;
	}
	MujocoHost host(argv[1]);
	if (!host.load_human()) {
		std::fprintf(stderr, "the guest could not build the human model\n");
		return 1;
	}
	host.prepare();
	const std::vector<std::string> names = host.body_names();
	ei_init();
	ei_cnode ec;
	if (ei_connect_init(&ec, argv[2], cookie, 0) < 0) {
		std::fprintf(stderr, "ei_connect_init failed\n");
		return 1;
	}
	int port = 0;
	const int listener = listen_on_any_port(port);
	if (listener < 0 || ei_publish(&ec, port) < 0) {
		std::fprintf(stderr, "could not listen or publish with epmd (is epmd running?)\n");
		return 1;
	}
	std::printf("ready %s port %d nq %d bodies %d\n", ei_thisnodename(&ec), port, host.cached_nq(), host.cached_nbody());
	std::fflush(stdout);
	for (;;) {
		ErlConnect conn;
		const int fd = ei_accept(&ec, listener, &conn);
		if (fd < 0) {
			continue;
		}
		const bool stop = serve(fd, host, names);
		close(fd);
		if (stop) {
			break;
		}
	}
	close(listener);
	return 0;
}
