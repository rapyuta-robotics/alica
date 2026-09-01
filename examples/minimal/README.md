# Minimal ALICA example

A self-contained, non-ROS ALICA agent: ~150 lines of C++ plus five model files.
It exercises the core of the framework — model loading, task allocation, team
discovery, behaviour execution, transition conditions, terminal-state success
and the blackboard — without Boost, ROS or the dynamic-loading machinery.

## What it does

`MinimalMaster.pml` has one entrypoint (`DefaultTask`, cardinality 1–2) pointing
at state `Working`, which runs the `CountUp` behaviour at 2 Hz. `CountUp` bumps a
counter on the global blackboard. The `CountReached` transition condition fires
once the counter hits 6, moving the agent into the `Finished` success state,
which stops the behaviour.

Run it with two agents and they discover each other through the dummy
communicator and share the entrypoint.

## Build

The top-level build includes this automatically:

```bash
cmake -S ../.. -B ../../build     # from examples/minimal
cmake --build ../../build -j"$(nproc)"
```

Disable it with `-DALICA_BUILD_EXAMPLES=OFF`.

To build it standalone instead, against an already-installed ALICA:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=<alica-install-prefix>
cmake --build build -j"$(nproc)"
```

## Run

Run it from this directory, so the relative `etc` path resolves:

```bash
# built by the top-level CMakeLists
LD_LIBRARY_PATH=../../build/lib \
  ../../build/bin/minimal_alica etc 2

# built standalone
./build/minimal_alica etc 2      # config folder, number of agents
```

Expected output (interleaved with engine logs):

```
started agentA (id 1000)
started agentB (id 1001)
[CountUp] started
[CountUp] agent 1000 counter = 1 (team size 1)
[CountUp] agent 1001 counter = 1 (team size 2)
...
[CountUp] agent 1000 counter = 6 (team size 2)
[CountUp] stopped
agentA final counter = 6
agentB final counter = 6
```

## Profiling with Tracy

The example carries optional [Tracy](https://github.com/wolfpld/tracy)
instrumentation: zones around agent construction and around each `CountUp`
tick, plus the counter as a plotted value. It is off by default and compiles to
nothing when off (see `src/Profiling.h`).

Tracy has two halves. Conan supplies the **client** -- the library that is
linked into the profiled process. It does not package the **server** -- the
`tracy-profiler` GUI or the `tracy-capture` CLI that records a trace -- so that
has to be built from Tracy's own sources once.

### 1. Build the example with instrumentation

```bash
conan install . --build=missing -o with_profile=True   # from the repo root
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake
cmake --build build --target minimal_alica -j"$(nproc)"
```

`conan install` passes `with_profile` through as `ALICA_BUILD_PROFILE`; the
configuration summary prints `Tracy profiling : ON` when it took. To build
without instrumentation, pass `-o with_profile=False` (or configure with
`-DALICA_BUILD_PROFILE=OFF`).

Linking the Tracy client is not free of side effects: it starts a profiler
thread and listens on TCP port 8086 from process start.

### 2. Build a server

Follow tracy official documentation for the guide on how to build and run tracy capture and tracy profiler tools

### 3. Record a trace

The client streams to a server over TCP; nothing is written to disk by the
profiled process itself. The example only runs for six seconds and would
normally exit before a server ever connected, so start it with
`TRACY_NO_EXIT=1`, which makes it block at exit until the trace has been
drained:

```bash
cd examples/minimal
LD_LIBRARY_PATH=../../build/lib TRACY_NO_EXIT=1 ../../build/bin/minimal_alica etc 2 &
/tmp/tracy-capture/build/tracy-capture -o minimal.tracy -f
```

```
Connecting to 127.0.0.1:8086...
Frames: 2
Time span: 6.26 s
Zones: 35
Saving trace... done!
```

Open `minimal.tracy` in the GUI. `TRACY_PORT` overrides 8086 on both sides if
it is taken; `tracy-capture -a <host>` records from another machine.

### Profiling the engine, not just the example

The zones here are all in the example's own sources. Instrumenting
`alica_engine` -- `PlanBase::run()` is the interesting one, it is the engine's
tick -- means the zones live in a shared library while `main` lives in the
executable. Tracy's client is a static library by default, which would give
each of them its own profiler instance and its own listening socket. Request a
shared one:

```bash
conan install . --build=missing -o with_profile=True -o "tracy/*:shared=True"
```

## Layout

| Path | Role |
| --- | --- |
| `etc/Alica.yaml` | engine configuration (frequencies, timeouts, discovery) |
| `etc/plans/MinimalMaster.pml` | the master plan: states, entrypoint, transition |
| `etc/plans/behaviours/CountUp.beh` | behaviour declaration (frequency, id) |
| `etc/plans/conditions/ConditionRepository.cnd` | transition-condition declarations |
| `etc/tasks/TaskRepository.tsk` | task definitions |
| `etc/roles/Roleset.rst` | roles and their task priorities |
| `src/CountUp.{h,cpp}` | the behaviour's domain-specific code |
| `src/Creators.h` | maps model ids to C++ objects |
| `src/main.cpp` | builds the contexts and starts the engines |

The numeric ids in the model files are what `src/Creators.h` switches on. The
Plan Designer normally generates these; here they are small hand-picked numbers
(`4001` = `CountUp`, `3002` = `CountReached`) to keep the mapping readable.
