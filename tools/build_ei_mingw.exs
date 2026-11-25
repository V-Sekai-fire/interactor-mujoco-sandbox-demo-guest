{opts, _, _} = OptionParser.parse(System.argv(), strict: [llvm_mingw: :string, otp_zip: :string, out: :string])
usage = "elixir tools/build_ei_mingw.exs --llvm-mingw <llvm-mingw bin> --otp-zip <otp_win64_29.1.2.zip> --out <dir>"
llvm = opts[:llvm_mingw] || (IO.puts(:stderr, usage); System.halt(2))
zip = opts[:otp_zip] || (IO.puts(:stderr, usage); System.halt(2))
out = Path.expand(opts[:out] || (IO.puts(:stderr, usage); System.halt(2)))
tag = "OTP-29.1.2"
ei = Path.join(out, "otp/lib/erl_interface-5.8.2")
File.rm_rf!(out)
File.mkdir_p!(Path.join(out, "obj"))
File.mkdir_p!(Path.join(out, "erts"))
File.mkdir_p!(Path.join(out, "cfg"))

{:ok, listing} = :zip.list_dir(String.to_charlist(zip))
wanted = for {:zip_file, name, _, _, _, _} <- listing, n = List.to_string(name), String.starts_with?(n, "lib/erl_interface-5.8.2/src/") or String.starts_with?(n, "lib/erl_interface-5.8.2/include/"), do: name
{:ok, _} = :zip.unzip(String.to_charlist(zip), file_list: wanted, cwd: String.to_charlist(Path.join(out, "otp")))

for f <- ["erl_md5.c", "erl_md5.h"] do
  url = "https://raw.githubusercontent.com/erlang/otp/#{tag}/erts/emulator/beam/#{f}"
  {_, 0} = System.cmd("curl", ["-sfL", "-o", Path.join(out, "erts/" <> f), url])
end

File.write!(Path.join(out, "cfg/config.h"), "#define SIZEOF_VOID_P 8\n#define SIZEOF_INT 4\n#define SIZEOF_LONG 4\n#define SIZEOF_LONG_LONG 8\ntypedef int socklen_t;\n#define HAVE_UNALIGNED_WORD_ACCESS 1\n")

dirs = ~w(connect decode encode epmd global misc)
sources = (for d <- dirs, f <- Path.wildcard(Path.join([ei, "src", d, "*.c"])), do: f) ++ [Path.join(out, "erts/erl_md5.c")]
includes = ["-I" <> Path.join(out, "erts"), "-I" <> Path.join(out, "cfg"), "-I" <> Path.join(ei, "include")] ++ for(d <- dirs, do: "-I" <> Path.join([ei, "src", d]))
defines = ~w(-DWIN32_THREADS -D_REENTRANT -D_WIN32_WINNT=0x0600 -DWINVER=0x0600 -D__WIN32__ -DERL_INTERFACE -DEI_NO_DEPR_WARN -include process.h)
cc = Path.join(llvm, "x86_64-w64-mingw32-clang")

results =
  sources
  |> Task.async_stream(
    fn src ->
      obj = Path.join([out, "obj", String.replace(Path.relative_to(src, out), "/", "_") <> ".o"])
      {text, code} = System.cmd(cc, ["-O2", "-w", "-c"] ++ includes ++ defines ++ [src, "-o", obj], stderr_to_stdout: true)
      {src, obj, code, text}
    end,
    timeout: :infinity
  )
  |> Enum.map(fn {:ok, r} -> r end)

failed = Enum.filter(results, fn {_, _, code, _} -> code != 0 end)
for {src, _, _, text} <- failed, do: IO.puts(:stderr, "failed: #{src}\n#{String.slice(text, 0, 400)}")
if failed != [], do: System.halt(1)

lib = Path.join(out, "libei.a")
{_, 0} = System.cmd(Path.join(llvm, "llvm-ar"), ["rcs", lib] ++ Enum.map(results, fn {_, obj, _, _} -> obj end))
IO.puts("wrote #{lib} #{File.stat!(lib).size} bytes from #{length(sources)} sources; headers in #{Path.join(ei, "include")}")
