#pragma once
#include <libriscv/machine.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

using Machine64 = riscv::Machine<riscv::RISCV64>;

constexpr int HEAP_SYSCALLS_BASE = 480;
constexpr int MEMORY_SYSCALLS_BASE = 485;
constexpr int ECALL_PRINT = 500;
constexpr int ECALL_SANDBOX_ADD = 547;
constexpr int ECALL_MATH_OP32 = 533;
constexpr int ECALL_MATH_OP64 = 534;
constexpr std::size_t STACK_SIZE = 2u << 20;
constexpr std::size_t ARENA_RESERVE = 256u << 10;
constexpr std::uint64_t START_INSTRUCTIONS = 4000000ull << 20;
constexpr std::uint64_t CALL_INSTRUCTIONS = 4000000ull << 20;

inline std::vector<std::uint8_t> read_file(const std::string &path) {
	std::ifstream f(path, std::ios::binary);
	if (!f) {
		throw std::runtime_error("cannot open " + path);
	}
	return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(f), {});
}

template <typename Float>
static void math_op(Machine64 &m) {
	const int op = (int)m.cpu.reg(10);
	const Float a = m.cpu.registers().getfl(10).template get<Float>();
	const Float b = m.cpu.registers().getfl(11).template get<Float>();
	switch (op) {
		case 0: m.set_result(Float(std::sin(a))); break;
		case 1: m.set_result(Float(std::cos(a))); break;
		case 2: m.set_result(Float(std::tan(a))); break;
		case 3: m.set_result(Float(std::asin(a))); break;
		case 4: m.set_result(Float(std::acos(a))); break;
		case 5: m.set_result(Float(std::atan(a))); break;
		case 6: m.set_result(Float(std::atan2(a, b))); break;
		case 7: m.set_result(Float(std::pow(a, b))); break;
		default:
			std::fprintf(stderr, "mujoco host: math operation %d is not implemented, stopping\n", op);
			m.stop();
	}
}

class MujocoHost {
public:
	std::vector<int> unknown;

	explicit MujocoHost(const std::string &elf, unsigned memory_mib = 1024) : binary(read_file(elf)) {
		seen = &unknown;
		Machine64::on_unhandled_syscall = [](Machine64 &m, std::size_t number) {
			seen->push_back((int)number);
			std::fprintf(stderr, "mujoco host: unhandled ecall %zu at pc 0x%llx, stopping\n", number, (unsigned long long)m.cpu.pc());
			m.stop();
		};
		Machine64::install_syscall_handler(ECALL_PRINT, [](Machine64 &m) {
			const auto text = m.memory.memstring(m.cpu.reg(10), (std::size_t)m.cpu.reg(11));
			std::fprintf(stderr, "guest: %s\n", text.c_str());
		});
		Machine64::install_syscall_handler(ECALL_MATH_OP32, math_op<float>);
		Machine64::install_syscall_handler(ECALL_MATH_OP64, math_op<double>);
		Machine64::install_syscall_handler(ECALL_SANDBOX_ADD, [](Machine64 &m) { m.set_result(0); });
		machine = new Machine64(binary, riscv::MachineOptions<riscv::RISCV64>{ .memory_max = std::uint64_t(memory_mib) << 20, .stack_size = STACK_SIZE });
		const std::uint64_t arena = machine->memory.memory_arena_size();
		const std::uint64_t mmap_now = machine->memory.mmap_address();
		std::uint64_t heap = (std::uint64_t)(arena * 0.8) & ~std::uint64_t(0xFFF);
		if (arena > mmap_now + ARENA_RESERVE) {
			heap = std::min<std::uint64_t>(heap, (arena - mmap_now - ARENA_RESERVE) & ~std::uint64_t(0xFFF));
		}
		const std::uint64_t area = machine->memory.mmap_allocate(heap);
		machine->setup_native_heap(HEAP_SYSCALLS_BASE, area, heap);
		machine->setup_native_memory(MEMORY_SYSCALLS_BASE);
		machine->setup_linux_syscalls();
		machine->setup_posix_threads();
		machine->setup_linux({ "mujoco" }, { "LC_CTYPE=C", "LC_ALL=C", "TZ=UTC", "LD_LIBRARY_PATH=" });
		machine->simulate(START_INSTRUCTIONS);
	}

	~MujocoHost() { delete machine; }

	bool load_human() { return call("mjc_raw_load_human") == 1; }
	int nq() { return (int)call("mjc_raw_nq"); }
	int nbody() { return (int)call("mjc_raw_nbody"); }

	std::vector<std::string> body_names() {
		const std::uint64_t cap = 4096;
		const std::uint64_t buf = machine->arena().malloc(cap);
		const int need = (int)call("mjc_raw_body_names", buf, cap);
		std::vector<char> raw((std::size_t)need);
		machine->copy_from_guest(raw.data(), buf, std::min<std::size_t>((std::size_t)need, cap));
		machine->arena().free(buf);
		std::vector<std::string> names;
		for (std::size_t at = 0; at < raw.size();) {
			names.emplace_back(&raw[at]);
			at += names.back().size() + 1;
		}
		return names;
	}

	bool fk(const double *qpos, int nq_count, double *out, int body_count) {
		if (nq_count != cached_nq() || body_count != cached_nbody()) {
			return false;
		}
		machine->copy_to_guest(qbuf, qpos, sizeof(double) * (std::size_t)nq_count);
		if ((int)call("mjc_raw_fk", qbuf, obuf) != body_count) {
			return false;
		}
		machine->copy_from_guest(out, obuf, sizeof(double) * 7 * (std::size_t)body_count);
		return true;
	}

	bool trip_unknown_ecall(int number) {
		const std::size_t before = unknown.size();
		Machine64::on_unhandled_syscall(*machine, (std::size_t)number);
		return unknown.size() == before + 1 && unknown.back() == number && machine->stopped();
	}

	void prepare() {
		nq_n = nq();
		nbody_n = nbody();
		qbuf = machine->arena().malloc(sizeof(double) * (std::size_t)nq_n);
		obuf = machine->arena().malloc(sizeof(double) * 7 * (std::size_t)nbody_n);
	}

	int cached_nq() const { return nq_n; }
	int cached_nbody() const { return nbody_n; }

private:
	template <typename... Args>
	std::uint64_t call(const char *name, Args... args) {
		return machine->vmcall<CALL_INSTRUCTIONS>(name, args...);
	}

	std::vector<std::uint8_t> binary;
	Machine64 *machine = nullptr;
	std::uint64_t qbuf = 0;
	std::uint64_t obuf = 0;
	int nq_n = 0;
	int nbody_n = 0;
	static inline std::vector<int> *seen = nullptr;
};
