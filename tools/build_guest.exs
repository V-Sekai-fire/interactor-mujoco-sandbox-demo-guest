{opts, _, _} = OptionParser.parse(System.argv(), strict: [llvm_bin: :string, sandbox_api: :string, out: :string])
need = fn key -> opts[key] || (IO.puts(:stderr, "missing --#{String.replace(to_string(key), "_", "-")}"); System.halt(2)) end
llvm = need.(:llvm_bin)
api = need.(:sandbox_api)
out = Path.expand(need.(:out))
root = Path.expand("..", __DIR__)
build = Path.join(System.tmp_dir!(), "mujoco_guest_build_#{System.unique_integer([:positive])}")

run = fn cmd, args ->
  case System.cmd(cmd, args, env: [{"PATH", llvm <> ":" <> System.get_env("PATH")}], stderr_to_stdout: true) do
    {_, 0} -> :ok
    {text, code} -> IO.puts(:stderr, "failed (#{code}): #{cmd} #{Enum.join(Enum.take(args, 2), " ")}\n#{String.slice(text, -1500, 1500)}"); System.halt(1)
  end
end

{version, 0} = System.cmd(Path.join(llvm, "clang"), ["--version"])
unless version =~ ~r/clang version 2[3-9]\./, do: (IO.puts(:stderr, "need clang 23 or newer, found: " <> hd(String.split(version, "\n"))); System.halt(1))

run.("cmake", ["-S", Path.join(root, "guest"), "-B", build, "-DCMAKE_TOOLCHAIN_FILE=" <> Path.join(root, "third_party/riscv64-sysroot/toolchain.cmake"), "-DDOUBLE_PRECISION=ON", "-DCMAKE_BUILD_TYPE=Release", "-DSANDBOX_API=" <> Path.expand(api)])
run.("cmake", ["--build", build, "-j", to_string(System.schedulers_online())])
File.mkdir_p!(Path.dirname(out))
File.cp!(Path.join(build, "mjstep"), out)
File.rm_rf!(build)
sha = :crypto.hash(:sha256, File.read!(out)) |> Base.encode16(case: :lower)
IO.puts("wrote #{out} #{File.stat!(out).size} bytes sha256 #{sha}")
