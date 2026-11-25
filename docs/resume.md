# Resume: the guest outside Godot, and its Windows build

State on 2026-11-25. Main holds the raw entry points, the Erlang node, its AsyncAPI document and the build scripts (pull request #25, merged through the queue). Branch `dev/win64-ei` holds the first step of the Windows build. The beta step this serves is RFD66606-2436: ANNY posed by this guest.

## Done

- `guest/main.cpp` exports `mjc_raw_*` entry points that take plain pointers, for a host that is not Godot.
- `cnode/` runs the guest ELF in libriscv as an Erlang C node on the shared message shape (`{command, ref, arg}` in, `{ref, {ok, term} | {error, binary}}` out; commands ping, info, call, fk, batch, cancel, stop). `cnode/asyncapi.yaml` describes it and passes the AsyncAPI checker of interactor-hair-cards-viewer; its commands and error reasons equal the code's.
- `host_probe` (all-frame hair gate, four controls) and `cnode/shape_check.exs` (16 checks, 6 of them refusals) pass on Linux.
- `tools/build_guest.exs` and `tools/build_cnode.exs`; a clean guest build had the same sha256 as the tested one.
- On `dev/win64-ei`: `tools/build_ei_mingw.exs` builds `libei.a` from the erl_interface sources in OTP's Windows zip with llvm-mingw. From a clean directory a test program linked with it runs on Windows and exits 0. OTP's own MSVC `ei.lib` and `ei_md.lib` do not link with lld.

## Next, in order

1. Port the node's sockets to winsock (`WSAStartup`, `closesocket`, no `arpa/inet.h`) and build `mujoco_cnode` for `x86_64-w64-mingw32` against that `libei.a` and a mingw build of libriscv; run `shape_check.exs` against it on Windows. Nothing here is tested on Windows beyond `ei` itself: no epmd connection, no handshake.
2. Measure the node with libriscv's own binary translation, interpreted against translated (interpreted: 1.2 to 1.5 ms a pose on Linux).
3. A NixOS flake that builds the same two things.
4. Add an `x-state-machine` block to `cnode/asyncapi.yaml` once the checker validates it (states: ready, stopped; `stop` is the only transition).
5. Run `release.yml` once on a tag; it has never run.

## Check it

    elixir tools/build_guest.exs --llvm-bin <clang 23.1.1 bin> --sandbox-api <contract-guest-runtime>/vendor/sandbox-api --out mujoco.human.double.elf
    elixir tools/build_cnode.exs --libriscv <godot-sandbox fork>/ext/libriscv --out build-cnode
    build-cnode/host_probe mujoco.human.double.elf <fixture prefix>
    elixir cnode/shape_check.exs --node build-cnode/mujoco_cnode --elf mujoco.human.double.elf --walk <fixture prefix>
    elixir tools/build_ei_mingw.exs --llvm-mingw <llvm-mingw bin> --otp-zip otp_win64_29.1.2.zip --out <dir>

## Machine-specific

- The toolchain was a pixi environment with clang, clangxx, lld and llvm-tools 23.1.1 (`tools/toolchain/pixi.toml`), because `libmujoco.a` holds LLVM 23 bitcode; clang 21 fails to link it.
- libriscv came from the godot-sandbox fork (V-Sekai-fire/godot-sandbox, commit 9fadcba, `ext/libriscv`), built with `RISCV_SYSCALLS_MAX=600`; the double guest needs contract-guest-runtime's `vendor/sandbox-api`.
- erl_interface 5.8.2 for Linux came from the pixi Elixir environment; for Windows, from `otp_win64_29.1.2.zip` (github.com/erlang/otp, tag OTP-29.1.2). llvm-mingw was release 20261117 with the UCRT runtime, running on Linux and cross-compiling.
- `epmd` must be running for the node to publish. The cookie is the `MUJOCO_COOKIE` environment variable.
- GitHub tokens come from `bao-mint-github --owner V-Sekai-fire --token-only`, with the header scoped to the organisation's URL; it sometimes answers HTTP 500, so retry.

## Files not in git

| File | Size | sha256 |
| --- | --- | --- |
| mujoco.human.double.elf (guest commit 9a4d3ac, Variant entry points only) | 6708368 | 4963e935b97d4a9a8811e72b27323b13eceb507237e48ec3c6b12038e60f7219 |
| mujoco.human.raw.double.elf (main, with the raw entry points) | 6709784 | c40f3e876b8d78073e2fbdb38fc834d29334b9e233d9319999b83c75134ce53c |
| walk_msh700.qpos.f64 (fixture for host_probe and shape_check) | 553152 | 1fe2d479720d41135c8b568063ac4f41552dc2dbfb3fc6372646fd6c0f175479 |
| walk_msh700.bodies.f64 (the reference body placements) | 3646944 | 8b71db1427e371557f8cad6d6f924ced2998acf4b34a48a9976df74af5742de5 |

Both ELFs are in the guest-builds journal of datasource-inventory. The two fixture files are made from a retargeted walk by `tools/muscle_fixture.exs` of persona-stream-model (see its `docs/resume.md` for the full recipe); the redistribution rights of the clip they come from were not checked, so do not publish them.
