{opts, _, _} = OptionParser.parse(System.argv(), strict: [libriscv: :string, out: :string, erl_interface: :string])
need = fn key -> opts[key] || (IO.puts(:stderr, "missing --#{String.replace(to_string(key), "_", "-")}"); System.halt(2)) end
libriscv = Path.expand(need.(:libriscv))
out = Path.expand(need.(:out))
root = Path.expand("../cnode", __DIR__)

erl_interface =
  opts[:erl_interface] ||
    case System.cmd("erl", ["-noshell", "-eval", "io:format(\"~s\", [code:lib_dir(erl_interface)]), halt()."]) do
      {dir, 0} when dir != "" -> dir
      _ -> IO.puts(:stderr, "pass --erl-interface, erl did not answer"); System.halt(2)
    end

run = fn args ->
  case System.cmd("cmake", args, stderr_to_stdout: true) do
    {_, 0} -> :ok
    {text, code} -> IO.puts(:stderr, "cmake failed (#{code})\n#{String.slice(text, -1500, 1500)}"); System.halt(1)
  end
end

run.(["-S", root, "-B", out, "-DLIBRISCV_DIR=" <> libriscv, "-DERL_INTERFACE_DIR=" <> erl_interface, "-DCMAKE_BUILD_TYPE=Release"])
run.(["--build", out, "-j", to_string(System.schedulers_online()), "--target", "mujoco_cnode", "host_probe"])

for name <- ["mujoco_cnode", "host_probe"] do
  path = Path.join(out, name)
  IO.puts("#{path} #{File.stat!(path).size} bytes sha256 " <> (:crypto.hash(:sha256, File.read!(path)) |> Base.encode16(case: :lower)))
end
