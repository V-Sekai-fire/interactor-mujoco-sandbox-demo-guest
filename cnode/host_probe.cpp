#include "mujoco_host.h"

#include <chrono>
#include <cmath>
#include <fstream>

static std::vector<double> read_f64(const std::string &path) {
	const auto bytes = read_file(path);
	std::vector<double> v(bytes.size() / sizeof(double));
	std::memcpy(v.data(), bytes.data(), v.size() * sizeof(double));
	return v;
}

int main(int argc, char **argv) {
	if (argc != 3) {
		std::fprintf(stderr, "usage: %s <mujoco ELF> <fixture prefix>\n", argv[0]);
		return 2;
	}
	using clock = std::chrono::steady_clock;
	auto t0 = clock::now();
	MujocoHost host(argv[1]);
	auto t1 = clock::now();
	if (!host.load_human()) {
		std::fprintf(stderr, "the guest could not build the human model\n");
		return 1;
	}
	auto t2 = clock::now();
	host.prepare();
	const int nq = host.cached_nq(), nbody = host.cached_nbody();
	const auto names = host.body_names();
	std::printf("startup %.0f ms, model build %.0f ms, nq %d, bodies %d, names %zu, first %s\n",
			std::chrono::duration<double, std::milli>(t1 - t0).count(), std::chrono::duration<double, std::milli>(t2 - t1).count(), nq, nbody, names.size(), names.empty() ? "?" : names[0].c_str());
	const std::string prefix = argv[2];
	const auto qpos = read_f64(prefix + ".qpos.f64");
	const auto ref = read_f64(prefix + ".bodies.f64");
	const int frames = (int)(qpos.size() / (std::size_t)nq);
	std::vector<double> out((std::size_t)nbody * 7);
	double worst_m = 0.0;
	double worst_q = 0.0;
	auto t3 = clock::now();
	for (int f = 0; f < frames; f++) {
		if (!host.fk(&qpos[(std::size_t)f * nq], nq, out.data(), nbody)) {
			std::fprintf(stderr, "fk failed at frame %d\n", f);
			return 1;
		}
		for (int b = 0; b < nbody; b++) {
			const double *r = &ref[((std::size_t)f * nbody + b) * 7];
			const double *g = &out[(std::size_t)b * 7];
			for (int k = 0; k < 3; k++) {
				worst_m = std::max(worst_m, std::fabs(r[k] - g[k]));
			}
			double dot = 0.0;
			for (int k = 0; k < 4; k++) {
				dot += r[3 + k] * g[3 + k];
			}
			worst_q = std::max(worst_q, 2.0 * std::acos(std::min(1.0, std::fabs(dot))) * 180.0 / M_PI);
		}
	}
	auto t4 = clock::now();
	std::printf("frames %d, worst position error %.9f m, worst rotation error %.9f deg, %.3f ms per pose\n", frames, worst_m, worst_q,
			std::chrono::duration<double, std::milli>(t4 - t3).count() / frames);
	const std::size_t unknown_before = host.unknown.size();
	int failures = 0;
	auto check = [&](bool ok, const char *what) {
		std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
		failures += ok ? 0 : 1;
	};
	check(frames == 804 && worst_m <= 1e-4 && worst_q <= 0.01, "every frame of the walk is inside the hair gate: 0.1 mm and 0.01 deg");
	check(unknown_before == 0, "the call path needs no ecall the host lacks");
	std::vector<double> bent(qpos.begin(), qpos.begin() + nq);
	bent[0] += 0.001;
	host.fk(bent.data(), nq, out.data(), nbody);
	double moved = 0.0;
	for (int b = 0; b < nbody; b++) {
		for (int k = 0; k < 3; k++) {
			moved = std::max(moved, std::fabs(out[(std::size_t)b * 7 + k] - ref[(std::size_t)b * 7 + k]));
		}
	}
	check(moved > 1e-4, "control: a pose moved 1 mm fails the hair gate");
	check(!host.fk(qpos.data(), nq - 1, out.data(), nbody), "a qpos of the wrong size is refused");
	check(host.trip_unknown_ecall(999), "an ecall the host does not have is logged by number and stops the guest");
	std::printf("PROBE: %d checks failed\n", failures);
	return failures == 0 ? 0 : 1;
}
