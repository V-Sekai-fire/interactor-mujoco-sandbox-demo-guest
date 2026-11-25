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
#include <string>

constexpr std::size_t MAX_OUT_BYTES = 64u << 20;
constexpr std::size_t MAX_ARGS = 6;

struct Node {
	MujocoHost *host;
	std::vector<std::string> names;
	bool stop = false;
};

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
	if (bind(fd, (sockaddr *)&addr, sizeof(addr)) < 0 || listen(fd, 5) < 0) {
		close(fd);
		return -1;
	}
	socklen_t len = sizeof(addr);
	getsockname(fd, (sockaddr *)&addr, &len);
	port = ntohs(addr.sin_port);
	return fd;
}

static bool is_atom(int type) {
	return type == ERL_ATOM_EXT || type == ERL_SMALL_ATOM_EXT || type == ERL_ATOM_UTF8_EXT || type == ERL_SMALL_ATOM_UTF8_EXT;
}

static bool is_integer(int type) {
	return type == ERL_SMALL_INTEGER_EXT || type == ERL_INTEGER_EXT || type == ERL_SMALL_BIG_EXT || type == ERL_LARGE_BIG_EXT;
}

static bool read_binary(const char *buf, int &index, std::vector<std::uint8_t> &out) {
	int type = 0, size = 0;
	if (ei_get_type(buf, &index, &type, &size) < 0 || type != ERL_BINARY_EXT) {
		return false;
	}
	out.resize((std::size_t)size);
	long got = 0;
	return ei_decode_binary(buf, &index, out.data(), &got) == 0;
}

static bool read_symbol(const char *buf, int &index, std::string &out) {
	int type = 0, size = 0;
	if (ei_get_type(buf, &index, &type, &size) < 0) {
		return false;
	}
	if (is_atom(type)) {
		char atom[MAXATOMLEN];
		if (ei_decode_atom(buf, &index, atom) < 0) {
			return false;
		}
		out = atom;
		return true;
	}
	std::vector<std::uint8_t> bytes;
	if (!read_binary(buf, index, bytes)) {
		return false;
	}
	out.assign(bytes.begin(), bytes.end());
	return true;
}

static std::string do_call(Node &node, const char *buf, int index, ei_x_buff &payload) {
	int arity = 0;
	std::string symbol;
	if (ei_decode_tuple_header(buf, &index, &arity) < 0 || arity != 2 || !read_symbol(buf, index, symbol)) {
		return "call_needs_symbol_and_arguments";
	}
	int count = 0;
	if (ei_decode_list_header(buf, &index, &count) < 0) {
		return "call_needs_an_argument_list";
	}
	const std::uint64_t address = node.host->symbol(symbol);
	if (address == 0) {
		return "unknown_symbol";
	}
	std::vector<std::uint64_t> args;
	std::vector<std::pair<std::uint64_t, std::size_t>> outs;
	std::vector<std::uint64_t> owned;
	std::string error;
	for (int i = 0; i < count && error.empty(); i++) {
		int type = 0, size = 0;
		ei_get_type(buf, &index, &type, &size);
		if (is_integer(type)) {
			long long value = 0;
			ei_decode_longlong(buf, &index, &value);
			args.push_back((std::uint64_t)value);
		} else if (type == ERL_SMALL_TUPLE_EXT) {
			int inner = 0;
			char tag[MAXATOMLEN];
			if (ei_decode_tuple_header(buf, &index, &inner) < 0 || inner != 2 || ei_decode_atom(buf, &index, tag) < 0) {
				error = "bad_argument";
			} else if (std::strcmp(tag, "in") == 0) {
				std::vector<std::uint8_t> bytes;
				if (!read_binary(buf, index, bytes)) {
					error = "in_needs_a_binary";
				} else {
					const std::uint64_t at = node.host->alloc(bytes.size() + 8);
					owned.push_back(at);
					node.host->write(at, bytes.data(), bytes.size());
					args.push_back(at);
				}
			} else if (std::strcmp(tag, "out") == 0) {
				long long bytes = 0;
				if (ei_decode_longlong(buf, &index, &bytes) < 0 || bytes < 0 || (std::size_t)bytes > MAX_OUT_BYTES) {
					error = "out_needs_a_size";
				} else {
					const std::uint64_t at = node.host->alloc((std::size_t)bytes + 8);
					owned.push_back(at);
					std::vector<std::uint8_t> zeros((std::size_t)bytes, 0);
					node.host->write(at, zeros.data(), zeros.size());
					outs.emplace_back(at, (std::size_t)bytes);
					args.push_back(at);
				}
			} else {
				error = "bad_argument";
			}
		} else {
			error = "bad_argument";
		}
	}
	if (error.empty() && args.size() > MAX_ARGS) {
		error = "too_many_arguments";
	}
	std::uint64_t returned = 0;
	if (error.empty()) {
		try {
			returned = node.host->invoke(address, args);
		} catch (const std::exception &) {
			error = "guest_fault";
		}
	}
	if (error.empty()) {
		ei_x_encode_tuple_header(&payload, 2);
		ei_x_encode_longlong(&payload, (long long)returned);
		ei_x_encode_list_header(&payload, (long)outs.size());
		for (const auto &o : outs) {
			std::vector<std::uint8_t> bytes(o.second);
			node.host->read(bytes.data(), o.first, o.second);
			ei_x_encode_binary(&payload, bytes.data(), (long)bytes.size());
		}
		ei_x_encode_empty_list(&payload);
	}
	for (std::uint64_t at : owned) {
		node.host->release(at);
	}
	return error;
}

static std::string do_fk(Node &node, const char *buf, int index, ei_x_buff &payload) {
	std::vector<std::uint8_t> qpos;
	if (!read_binary(buf, index, qpos)) {
		return "fk_needs_a_binary";
	}
	const int nq = node.host->cached_nq(), nbody = node.host->cached_nbody();
	if (qpos.size() != sizeof(double) * (std::size_t)nq) {
		return "bad_qpos_size";
	}
	std::vector<double> q((std::size_t)nq), bodies((std::size_t)nbody * 7);
	std::memcpy(q.data(), qpos.data(), qpos.size());
	if (!node.host->fk(q.data(), nq, bodies.data(), nbody)) {
		return "fk_failed";
	}
	ei_x_encode_binary(&payload, bodies.data(), (long)(bodies.size() * sizeof(double)));
	return "";
}

static std::string do_info(Node &node, ei_x_buff &payload) {
	static const char *commands[] = { "ping", "info", "call", "fk", "batch", "cancel", "stop" };
	ei_x_encode_map_header(&payload, 7);
	ei_x_encode_atom(&payload, "name");
	ei_x_encode_binary(&payload, "mujoco", 6);
	ei_x_encode_atom(&payload, "version");
	ei_x_encode_binary(&payload, "1", 1);
	ei_x_encode_atom(&payload, "commands");
	ei_x_encode_list_header(&payload, 7);
	for (const char *c : commands) {
		ei_x_encode_atom(&payload, c);
	}
	ei_x_encode_empty_list(&payload);
	ei_x_encode_atom(&payload, "batch");
	ei_x_encode_binary(&payload, "list", 4);
	ei_x_encode_atom(&payload, "nq");
	ei_x_encode_long(&payload, node.host->cached_nq());
	ei_x_encode_atom(&payload, "nbody");
	ei_x_encode_long(&payload, node.host->cached_nbody());
	ei_x_encode_atom(&payload, "bodies");
	ei_x_encode_list_header(&payload, (long)node.names.size());
	for (const std::string &n : node.names) {
		ei_x_encode_binary(&payload, n.data(), (long)n.size());
	}
	ei_x_encode_empty_list(&payload);
	return "";
}

static std::string run(Node &node, const std::string &command, const char *buf, int index, ei_x_buff &payload, bool nested);

static std::string do_batch(Node &node, const char *buf, int index, ei_x_buff &payload) {
	int arity = 0;
	std::string inner;
	if (ei_decode_tuple_header(buf, &index, &arity) < 0 || arity != 2 || !read_symbol(buf, index, inner) || inner == "batch") {
		return "batch_needs_a_command_and_a_list";
	}
	int count = 0;
	if (ei_decode_list_header(buf, &index, &count) < 0) {
		return "batch_needs_a_list";
	}
	std::vector<std::string> items;
	ei_x_buff body;
	ei_x_new(&body);
	std::string error;
	for (int i = 0; i < count && error.empty(); i++) {
		ei_x_buff one;
		ei_x_new(&one);
		error = run(node, inner, buf, index, one, true);
		ei_x_append_buf(&body, one.buff, one.index);
		ei_x_free(&one);
		ei_skip_term(buf, &index);
	}
	if (error.empty()) {
		ei_x_encode_list_header(&payload, count);
		ei_x_append_buf(&payload, body.buff, body.index);
		ei_x_encode_empty_list(&payload);
	}
	ei_x_free(&body);
	return error;
}

static std::string run(Node &node, const std::string &command, const char *buf, int index, ei_x_buff &payload, bool nested) {
	if (command == "ping") {
		ei_x_encode_atom(&payload, "pong");
		return "";
	}
	if (command == "info") {
		return do_info(node, payload);
	}
	if (command == "call") {
		return do_call(node, buf, index, payload);
	}
	if (command == "fk") {
		return do_fk(node, buf, index, payload);
	}
	if (command == "batch" && !nested) {
		return do_batch(node, buf, index, payload);
	}
	if (command == "cancel") {
		return "not cancellable";
	}
	if (command == "stop" && !nested) {
		node.stop = true;
		ei_x_encode_atom(&payload, "ok");
		return "";
	}
	return "unknown_command";
}

static void answer(int fd, erlang_pid &to, erlang_ref &ref, const std::string &error, ei_x_buff &payload) {
	ei_x_buff out;
	ei_x_new_with_version(&out);
	ei_x_encode_tuple_header(&out, 2);
	ei_x_encode_ref(&out, &ref);
	ei_x_encode_tuple_header(&out, 2);
	if (error.empty()) {
		ei_x_encode_atom(&out, "ok");
		ei_x_append_buf(&out, payload.buff, payload.index);
	} else {
		ei_x_encode_atom(&out, "error");
		ei_x_encode_binary(&out, error.data(), (long)error.size());
	}
	ei_send(fd, &to, out.buff, out.index);
	ei_x_free(&out);
}

static void serve(int fd, Node &node) {
	ei_x_buff in;
	ei_x_new(&in);
	while (!node.stop) {
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
		erlang_ref ref;
		std::string command;
		if (ei_decode_version(in.buff, &index, &version) < 0 || ei_decode_tuple_header(in.buff, &index, &arity) < 0 || arity != 3 ||
				!read_symbol(in.buff, index, command) || ei_decode_ref(in.buff, &index, &ref) < 0) {
			in.index = 0;
			continue;
		}
		ei_x_buff payload;
		ei_x_new(&payload);
		const std::string error = run(node, command, in.buff, index, payload, false);
		answer(fd, msg.from, ref, error, payload);
		ei_x_free(&payload);
		in.index = 0;
	}
	ei_x_free(&in);
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
	Node node{ &host, host.body_names() };
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
	while (!node.stop) {
		ErlConnect conn;
		const int fd = ei_accept(&ec, listener, &conn);
		if (fd < 0) {
			continue;
		}
		serve(fd, node);
		close(fd);
	}
	close(listener);
	return 0;
}
