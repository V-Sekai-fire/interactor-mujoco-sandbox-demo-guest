# interactor-mujoco-sandbox-demo-guest

A physics library built as a RISC-V guest program that an engine project loads into a sandbox and steps from script.

## What it is for

The host owns the tick: every step happens because the script asked for one, so a run repeats bit for bit on any machine. The guest answers stroke-crossing queries for the curve network and holds a scene's colliders for the ray and contact queries a walking player needs. RFD66606-2274 owns the crossings design.

## Build and run

    godot --path project --import
    godot --path project

The import registers the sandbox extension on a fresh clone.

To rebuild the guest, configure `guest/` with CMake, using `third_party/riscv64-sysroot/toolchain.cmake` as the toolchain file, and copy the built `mjstep` program to `project/plans/mujoco.elf`, the path the host loads.

## The guest ELF and the Erlang node

Two things are built from this repository, and neither is committed.

    elixir tools/build_guest.exs --llvm-bin <clang 23.1.1 bin> --sandbox-api <contract-guest-runtime>/vendor/sandbox-api --out mujoco.human.double.elf
    elixir tools/build_cnode.exs --libriscv <godot-sandbox fork>/ext/libriscv --out build-cnode

`build_guest.exs` makes the double-precision guest with the MS-Human-700 model. It needs clang, lld and llvm-ar 23.1.1 or newer, because `libmujoco.a` holds LLVM 23 bitcode; an older clang fails to link it. `tools/toolchain/pixi.toml` pins that toolchain. A run from a clean directory gave a guest with the same sha256 as the one that was tested.

`build_cnode.exs` makes `mujoco_cnode`, an Erlang C node that runs the same ELF with libriscv and no Godot, and `host_probe`, which compares its forward kinematics with reference body placements. The guest exports plain `mjc_raw_*` functions for this (load, `nq`, body count, forward kinematics only, body names), because the Godot entry points pass arrays as engine Variants that only Godot can hold.

Elixir talks to the node with `send({:any, node}, {self(), request})`: `:info` answers `{:ok, nq, nbody, names}`, `{:fk, <<qpos as float64>>}` answers `{:ok, <<bodies as float64>>}` (7 numbers a body: x y z, then w x y z), and `:stop` ends it. The cookie comes from `MUJOCO_COOKIE`. The node loads the model before it publishes, which takes several seconds.

Measured on one machine, 804 frames of a retargeted walk against Python's MuJoCo: 0.000000000 m and 7.4e-6 degrees at the worst, 1.2 to 1.5 ms a pose in the interpreter. The Godot path (`mjc_forward`, which also runs the muscle model) takes 18 ms interpreted and 3.7 to 4.6 ms binary translated. The node's own binary translation was not measured.

`.github/workflows/release.yml` builds both on a tag and attaches them with `SHA256SUMS`. It was written without being run.

## Vendored dependencies

Every dependency is a squashed git subtree, so a clone needs no submodule step and a refresh is one command:

    git subtree pull --prefix=<path> https://github.com/<repository> <ref> --squash

| Path | Repository | Ref | Licence |
|---|---|---|---|
| `third_party/doctest` | doctest/doctest | `master` (vendored at `ae7a135`) | MIT |
| `third_party/riscv64-mujoco` | V-Sekai-archive/repository-riscv64-mujoco | `main/main` | Apache-2.0 |
| `third_party/riscv64-sysroot` | V-Sekai-fire/repository-riscv64-sysroot | `main` | LGPL-2.1-or-later, GPL-3.0-with-GCC-exception |
| `third_party/sandbox-api` | V-Sekai-fire/godot-sandbox | `subtree/program-cpp` | BSD-3-Clause |
| `third_party/witness-cpp` | V-Sekai-fire/repository-witness-cpp | `main` | MIT |
| `project/addons/canvas_plane` | V-Sekai/canvas_plane | `main` | MIT |
| `project/addons/distance_field_outlines` | pink-arcana/godot-distance-field-outlines | `main` | MIT |
| `project/addons/godot_sandbox` | V-Sekai-fire/godot-sandbox | `subtree/bin-addons-godot-sandbox` | BSD-3-Clause |
| `project/addons/interaction_system` | V-Sekai-archive/entities-multiplayer-fabric-interaction-system | `main` | MIT |
| `project/addons/vsekai_godot_mcp` | V-Sekai-fire/transport-godot-mcp | `subtree/addons-vsekai-godot-mcp` | MIT |

The last `Squashed '<path>/'` commit on a path names the content that was taken.

Three refs are split branches of their repository rather than the repository itself, because the content sits in a subdirectory: `program/cpp`, `bin/addons/godot_sandbox` and `addons/vsekai_godot_mcp`. `git subtree split` cannot make those splits here, since it skips every commit after a subtree merge with a different prefix and those repositories carry such merges; the branch is made with `git filter-branch --subdirectory-filter <dir>` and pushed.

`third_party/sandbox-api` carries three changes of this repository's own on top of its subtree: the `.sandbox_variant` marker, the fork's syscall ids, and the dropped packed-array calls. A pull merges them rather than replacing them. That copy is behind the fork's `main`; moving it forward is a change of its own, because it rebuilds the committed guest programs. `project/addons/godot_sandbox` likewise carries what the release ships and the source directory does not hold: the built libraries, the compiler ELF, the licence files, the `.uid` sidecars, and the `linux.arm64` and macOS entries in `bin/godot-riscv.gdextension`.

## Licence

MIT. See [LICENSE](LICENSE). Vendored projects under `third_party/` carry their own licences.
