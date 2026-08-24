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

Requires the ALICA packages to be built and installed first (see the guide in
`docs/`). Then:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=<alica-install-prefix>
cmake --build build -j"$(nproc)"
```

## Run

```bash
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
