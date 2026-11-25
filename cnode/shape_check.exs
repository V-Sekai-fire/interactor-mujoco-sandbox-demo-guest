{opts, _, _} = OptionParser.parse(System.argv(), strict: [node: :string, elf: :string, walk: :string])
usage = "elixir cnode/shape_check.exs --node <mujoco_cnode> --elf <ELF> --walk <fixture prefix with .qpos.f64 and .bodies.f64>"
bin = opts[:node] || (IO.puts(:stderr, usage); System.halt(2))
elf = opts[:elf] || (IO.puts(:stderr, usage); System.halt(2))
walk = opts[:walk] || (IO.puts(:stderr, usage); System.halt(2))
cookie = :crypto.strong_rand_bytes(12) |> Base.encode16()
host = :net_adm.localhost() |> to_string() |> String.split(".") |> hd()
port = Port.open({:spawn_executable, bin}, [:binary, :stderr_to_stdout, args: [elf, "shape"], env: [{~c"MUJOCO_COOKIE", String.to_charlist(cookie)}]])

wait = fn wait ->
  receive do
    {^port, {:data, text}} -> if text =~ "ready", do: :ok, else: wait.(wait)
  after
    120_000 -> raise "the node did not become ready"
  end
end

wait.(wait)
{:ok, _} = Node.start(:"check_#{System.unique_integer([:positive])}@#{host}", :shortnames)
Node.set_cookie(String.to_atom(cookie))
target = :"shape@#{host}"
true = Node.connect(target)

ask = fn command, arg ->
  ref = make_ref()
  send({:mujoco, target}, {command, ref, arg})
  receive do
    {^ref, reply} -> reply
  after
    30_000 -> :no_reply
  end
end

failures = :counters.new(1, [])
check = fn ok, what ->
  unless ok, do: :counters.add(failures, 1, 1)
  IO.puts((if ok, do: "ok   ", else: "FAIL ") <> what)
end

qpos = File.read!(walk <> ".qpos.f64")
reference = File.read!(walk <> ".bodies.f64")
nq = 86 * 8
nb = 81 * 7 * 8
frame = fn f -> binary_part(qpos, f * nq, nq) end
expected = fn f -> binary_part(reference, f * nb, nb) end
worst = fn a, b -> for({<<x::little-float-64>>, <<y::little-float-64>>} <- Enum.zip(for(<<c::binary-8 <- a>>, do: c), for(<<c::binary-8 <- b>>, do: c)), do: abs(x - y)) |> Enum.max() end

check.(ask.(:ping, nil) == {:ok, :pong}, "ping answers {ok, pong}")
{:ok, info} = ask.(:info, nil)
check.(is_map(info) and info.name == "mujoco" and info.nq == 86 and info.nbody == 81 and length(info.bodies) == 81, "info carries name, nq, nbody and the body names")
check.(Enum.all?([:ping, :info, :call, :batch, :cancel], &(&1 in info.commands)) and info.batch == "list", "info lists its commands and its batch style")

{:ok, {returned, [bodies]}} = ask.(:call, {"mjc_raw_fk", [{:in, frame.(100)}, {:out, nb}]})
check.(worst.(bodies, expected.(100)) <= 1.0e-4, "call runs mjc_raw_fk and its out buffer is inside the hair gate")
check.(returned == 81, "call returns the function's integer result with the out buffers: the body count")
check.(ask.(:call, {"mjc_raw_nq", []}) == {:ok, {86, []}}, "call with no buffers returns just the result")
{:ok, same} = ask.(:fk, frame.(100))
check.(same == bodies, "the fk command and the generic call give the same bytes")
{:ok, many} = ask.(:batch, {:fk, [frame.(0), frame.(400), frame.(803)]})
check.(many == [expected_0 = elem(ask.(:fk, frame.(0)), 1), elem(ask.(:fk, frame.(400)), 1), elem(ask.(:fk, frame.(803)), 1)] and worst.(expected_0, expected.(0)) <= 1.0e-4, "batch gives one reply per item, equal to separate requests")
{:ok, {_, [names]}} = ask.(:call, {"mjc_raw_body_names", [{:out, 4096}]})
check.(String.starts_with?(names, "world\0pelvis\0"), "call reads the body names through an out buffer")

for {what, command, arg} <- [
      {"an unknown command", :nope, nil},
      {"an unknown symbol", :call, {"no_such_function", []}},
      {"a qpos of the wrong size", :fk, <<0, 0, 0>>},
      {"seven arguments", :call, {"mjc_raw_nq", [1, 2, 3, 4, 5, 6, 7]}},
      {"cancel on a node with no long jobs", :cancel, make_ref()},
      {"a batch inside a batch", :batch, {:batch, []}}
    ] do
  reply = ask.(command, arg)
  check.(match?({:error, r} when is_binary(r), reply), "#{what} is refused with an error binary")
end

check.(ask.(:ping, nil) == {:ok, :pong}, "the node still answers after the refused requests")
ask.(:stop, nil)
IO.puts("SHAPE: #{:counters.get(failures, 1)} checks failed")
System.halt(if :counters.get(failures, 1) == 0, do: 0, else: 1)
