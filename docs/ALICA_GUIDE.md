# The ALICA Framework — A Practical Guide

ALICA ("A Language for Interactive Cooperative Agents") is a C++ framework for
coordinating **teams** of autonomous agents. You describe team strategy
declaratively as a hierarchy of plans and state machines; at runtime every agent
independently runs the same program, and the engine continuously negotiates
**who does what** by exchanging beliefs with its teammates.

It was born in 2009 in the RoboCup Middle Size League and is maintained by
Rapyuta Robotics, who run it in production warehouses.

Everything in this guide was verified against commit `dfc62ec7` on `devel`,
built and run on **Ubuntu 22.04.5** with GCC 11.4 and CMake 3.22, **without ROS**.

---

## Table of contents

1. [Building on Ubuntu 22.04](#1-building-on-ubuntu-2204)
2. [Running the tests and samples](#2-running-the-tests-and-samples)
3. [A bare-minimum example](#3-a-bare-minimum-example)
4. [Full feature set](#4-full-feature-set)
5. [What is the "ALICA planner"?](#5-what-is-the-alica-planner)
6. [ALICA vs. off-the-shelf alternatives](#6-alica-vs-off-the-shelf-alternatives)

---

## 1. Building on Ubuntu 22.04

### 1.1 The key fact: the core is not a ROS project

This repository is a **multi-package workspace with no top-level `CMakeLists.txt`**.
Every package has its own `CMakeLists.txt` and its own `package.xml`, and the
core packages declare `<build_type>cmake</build_type>` — they are plain CMake
projects that merely *tolerate* being built by catkin/colcon.

That means **you can build the entire core with `cmake` directly and never touch
ROS.** ROS only enters through optional `supplementary/` packages. This is the
recommended path if you prefer CMake, and it is what this guide does.

Concretely, the packages split three ways:

| Group | Packages | Needs ROS? |
| --- | --- | --- |
| **Core** | `alica_solver_interface`, `alica_engine`, `alica_dummy_proxy`, `alica_simple_solver`, `alica_dynamic_loading`, `alica_standard_library`, `alica_test_utility`, `alica_tests` | **No** |
| **Supplementary, ROS-free** | `supplementary/autodiff`, `supplementary/constraintsolver`, `supplementary/alica_dummy_tracing` | **No** |
| **Supplementary, ROS** | `alica_msgs`, `alica_ros1/*`, `alica_ros2/*`, `alica_tracing`, `alica_turtlesim` | **Yes** |

### 1.2 Dependency list

**Non-ROS dependencies (all that the core needs):**

| Dependency | Ubuntu 22.04 package | Used by | Notes |
| --- | --- | --- | --- |
| C++17 compiler | `build-essential` | everything | GCC 11 is fine; `-std=c++17` is forced by `cmake_flags/cflags.cmake` |
| CMake ≥ 3.5.1 | `cmake` | everything | 3.10+ for `alica_tests` |
| yaml-cpp | `libyaml-cpp-dev` | `alica_engine`, `alica_tests`, `constraintsolver`, `alica_tracing` | reads `Alica.yaml` and all `.pml`/`.beh`/`.cnd`/`.tsk`/`.rst` model files (they are JSON, parsed as YAML) |
| Boost (system, filesystem, dll) | `libboost-all-dev` (or `libboost-system-dev libboost-filesystem-dev`) | `alica_dynamic_loading`, `alica_standard_library`, `alica_tests` | `boost::dll` loads behaviours/plans from shared libraries at runtime |
| GoogleTest | `libgtest-dev` | test targets only | `alica_solver_interface`, `alica_tests`, `autodiff`, `constraintsolver` |
| Doxygen (optional) | `doxygen` | docs | enable with `-DBUILD_DOXYGEN=ON` |

```bash
sudo apt update
sudo apt install -y build-essential cmake libyaml-cpp-dev libboost-all-dev libgtest-dev
```

**ROS dependencies (only for `supplementary/`):**

| Dependency | ROS 1 (Noetic) | ROS 2 (Humble — the Ubuntu 22.04 match) |
| --- | --- | --- |
| Client library | `roscpp` | `rclcpp` |
| Build tool | `catkin`, `python3-catkin-tools` | `ament_cmake`, `colcon` |
| Message generation | `message_generation`, `message_runtime` | `rosidl_default_generators`, `rosidl_default_runtime` |
| Messages | `std_msgs`, `geometry_msgs` | `std_msgs`, `geometry_msgs` |
| Demo | `turtlesim` | `turtlesim` |
| Test | `rostest` | — |

Note that **Noetic is not available on Ubuntu 22.04**. If you are on 22.04 and
want ROS, that means ROS 2 Humble — which is exactly what upstream CI tests
(`.github/workflows/ros2_build_test/Dockerfile` uses `ros:humble`).

### 1.3 Build order

`find_package()` calls impose a strict order. Each package must be **installed**
before its dependents configure:

```
alica_solver_interface
  └── alica_engine
        ├── alica_dummy_proxy
        ├── alica_simple_solver
        ├── alica_dynamic_loading      (+ Boost)
        ├── alica_standard_library     (+ Boost)
        ├── alica_test_utility         (needs alica_dummy_proxy)
        └── supplementary/autodiff → supplementary/constraintsolver
              └── alica_tests          (needs all of the above)
```

### 1.4 Build script (verified working)

```bash
#!/bin/bash
set -e
REPO=$PWD                 # run from the repository root
PREFIX=$HOME/alica-install
BUILD=$HOME/alica-build

build() {  # build() <source-dir>
  local name=$(basename "$1")
  cmake -S "$REPO/$1" -B "$BUILD/$name" \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_PREFIX_PATH="$PREFIX" \
        -DCMAKE_BUILD_TYPE=Release
  cmake --build "$BUILD/$name" -j"$(nproc)"
  cmake --install "$BUILD/$name"
}

build alica_solver_interface
build alica_engine
build alica_dummy_proxy
build alica_simple_solver
build alica_dynamic_loading
build alica_standard_library
build alica_test_utility
build supplementary/autodiff
build supplementary/constraintsolver
build alica_tests
```

Then, so that runtime dynamic loading and the shared libraries resolve:

```bash
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH
```

### 1.5 Gotchas worth knowing before you start

These are all things this guide hit in practice.

**`CMAKE_BUILD_TYPE` is overridden.** `alica_engine/CMakeLists.txt:5` and
`alica_tests/CMakeLists.txt:6` both hard-code `set(CMAKE_BUILD_TYPE Debug)`.
Passing `-DCMAKE_BUILD_TYPE=Release` to those two packages has no effect — they
always build Debug, which is why the installed export file is named
`alica_engineTargets-debug.cmake`. To get an optimised engine you have to edit
that line or override the flags directly.

**Do not build yaml-cpp from source unless you patch it.** ALICA links yaml-cpp
through the `${YAML_CPP_LIBRARIES}` *variable*. Upstream yaml-cpp 0.7.0's own
`yaml-cpp-config.cmake` sets that variable to an **empty string** — so the
libraries silently link nothing, and every final executable then fails with
hundreds of `undefined reference to YAML::...` errors. Debian/Ubuntu **patch**
this: `libyaml-cpp-dev` 0.7.0+dfsg-8build1 correctly sets
`set(YAML_CPP_LIBRARIES "yaml-cpp")`. So use the distro package. If you must
build from source, fix that one line in the installed config file — passing
`-DYAML_CPP_LIBRARIES=yaml-cpp` on the command line does **not** work, because
`find_package(yaml-cpp)` overwrites your value with the empty one.

**`LD_LIBRARY_PATH` is mandatory at runtime, not just at build time.**
`alica_dynamic_loading` resolves behaviour/plan implementations by reading
`LD_LIBRARY_PATH` directly (`DynamicLoadingUtils.cpp:calculateLibraryPath`) and
throws `DynamicLoadingException{"could not load LD_LIBRARY_PATH"}` if the
variable is unset — even if the library sits next to the binary.

**`-Werror` is on.** `cmake_flags/cflags.cmake` sets `-Werror` for C++ (with
`-Wno-error=deprecated-declarations`). A newer compiler than GCC 11 may surface
new warnings and break the build.

### 1.6 If you would rather use colcon (ROS 2)

Upstream CI does this, and it is the only sane route for the ROS packages:

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/rapyuta-robotics/alica.git
cd ~/ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y -r --from-paths src --ignore-src --rosdistro humble
colcon build --continue-on-error \
  --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim \
                  alica_tests supplementary_tests libalica-turtlesim
```

The skip list is copied from CI — those packages are ROS 1-only or otherwise
excluded there. For ROS 1 Noetic (on Ubuntu 20.04) CI uses `catkin build` with
`--skiplist alica_tracing alica_ros2_proxy alica_ros2_turtlesim`.

---

## 2. Running the tests and samples

### 2.1 The test suites

Tests are gated on `BUILD_TESTING` (default ON via `include(CTest)`) and
registered with `gtest_discover_tests`, so plain `ctest` works. Four packages
ship tests:

```bash
# The main suite — 92 tests covering the engine end to end
cd $BUILD/alica_tests
LD_LIBRARY_PATH=$BUILD/alica_tests:$PREFIX/lib ctest -j4 --output-on-failure

# The others
cd $BUILD/alica_solver_interface && ctest          #  2 tests
cd $BUILD/autodiff              && ctest          # 20 tests
cd $BUILD/constraintsolver      && LD_LIBRARY_PATH=$PREFIX/lib ctest   # 9 tests
```

Verified result on Ubuntu 22.04, no ROS: **123/123 pass** (92 + 2 + 20 + 9).

The `LD_LIBRARY_PATH` on the main suite is not optional — `alica_tests` builds
its behaviours into `libalica-tests.so` and loads them dynamically, exactly like
a real application.

You can also drive the gtest binary directly, which is handy for filtering:

```bash
$BUILD/alica_tests/alica_tests-test --gtest_list_tests
$BUILD/alica_tests/alica_tests-test --gtest_filter='TestBlackboard.*'
```

What the 92 tests cover is a good map of the engine's features:
task assignment and authority/conflict resolution, sync transitions, plan
parsing, blackboards (local, global, inherited), placeholders, failure
handling, scheduling, tracing, agent discovery and death, variable
synchronisation, config reloading, and success propagation.

Under catkin/colcon the equivalents are:

```bash
CTEST_OUTPUT_ON_FAILURE=1 GTEST_FILTER=-AlicaTurtlesimTest.* catkin run_tests
catkin_test_results --verbose
# or
colcon test && colcon test-result --verbose
```

CI excludes `AlicaTurtlesimTest.*` because those need a running turtlesim.

### 2.2 The samples

**The minimal non-ROS sample** — `examples/minimal/`, added by this guide. No
ROS, no Boost, no dynamic loading. See [section 3](#3-a-bare-minimum-example).

```bash
cd examples/minimal
cmake -S . -B build -DCMAKE_PREFIX_PATH=$PREFIX
cmake --build build -j"$(nproc)"
./build/minimal_alica etc 2
```

**The turtlesim sample** — the canonical tutorial, and the one to read to
understand how ALICA is used for real. Two or more turtles negotiate: the engine
assigns one the `Leader` task (drives to the centre) and the rest the `Follower`
task (arrange themselves on a circle around it, with positions computed by the
constraint solver). This *is* the multi-agent story, and it needs ROS.

ROS 2 (Humble):

```bash
source install/setup.bash
ros2 launch alica_ros2_turtlesim env.launch.xml           # turtlesim window
ros2 launch alica_ros2_turtlesim turtle.launch.xml turtles:=2
ros2 topic pub /init std_msgs/msg/Empty                   # start moving
```

ROS 1 (Noetic):

```bash
source devel/setup.bash
roslaunch alica_ros_turtlesim env.launch
roslaunch alica_ros_turtlesim turtle.launch turtles:=2
rostopic pub turtle1/join_formation std_msgs/Empty "{}"   # join
rostopic pub turtle1/leave_formation std_msgs/Empty "{}"  # and leave
```

Joining and leaving at runtime is the interesting part: watch the remaining
turtles re-negotiate the formation as the team composition changes.

> **Known snag** (documented in both turtlesim READMEs): if you see
> `Unknown quantifier type encountered!`, check that `quantifierType` is `"all"`
> in `etc/plans/Move.pml`.

---

## 3. A bare-minimum example

Full source: **`examples/minimal/`**. It is deliberately the smallest thing that
still demonstrates the core: model loading, task allocation, team discovery,
behaviour execution, a transition condition, terminal-state success, and the
blackboard — in ~150 lines of C++ plus five model files, with **no ROS, no Boost
and no dynamic loading**.

### 3.1 The idea

One entrypoint (`DefaultTask`, cardinality 1–2) points at state `Working`, which
runs a `CountUp` behaviour at 2 Hz. `CountUp` increments a counter on the global
blackboard. A transition condition `CountReached` fires when the counter hits 6,
moving the agent to the `Finished` success state, which stops the behaviour.
Start two agents and they discover each other and share the entrypoint.

### 3.2 An ALICA program is data + code

The declarative half lives in model files under `etc/`. They are **JSON, parsed
as YAML**, and normally authored by the Plan Designer — but they are perfectly
writable by hand, which is what the example does. The engine finds them by
**recursively scanning** the config folders you pass it, so the directory layout
is a convention rather than a requirement (the `PlanDir`/`RoleDir`/`TaskDir` keys
in `Alica.yaml` are vestigial and not read by the engine).

| File | Extension | Declares |
| --- | --- | --- |
| `MinimalMaster.pml` | `.pml` | the plan: states, entrypoints, transitions |
| `CountUp.beh` | `.beh` | a behaviour: its id, tick frequency, conditions |
| `ConditionRepository.cnd` | `.cnd` | transition conditions |
| `TaskRepository.tsk` | `.tsk` | tasks |
| `Roleset.rst` | `.rst` | roles and their per-task priorities |
| `Alica.yaml` | — | engine tuning: frequencies, timeouts, discovery |

Every element carries a numeric **id**, and ids are how the model refers to
code. Cross-file references use `<file>#<id>`:

```json
"task": "TaskRepository.tsk#1002",
"condition": "ConditionRepository.cnd#3002",
"abstractPlan": "CountUp.beh#4001"
```

Use the bare filename, not a path — `"behaviours/CountUp.beh#4001"` works but
logs `File name expected, however path is specified`.

The master plan, trimmed to essentials:

```json
{
  "id": 5001, "name": "MinimalMaster", "masterPlan": true,
  "entryPoints": [{
    "id": 5002, "task": "TaskRepository.tsk#1002",
    "minCardinality": 1, "maxCardinality": 2,
    "state": 5003, "successRequired": true
  }],
  "states": [
    { "id": 5003, "name": "Working", "type": "State",
      "entryPoint": 5002, "outTransitions": [5005],
      "confAbstractPlanWrappers": [{ "abstractPlan": "CountUp.beh#4001", "id": 5006 }] },
    { "id": 5004, "name": "Finished", "type": "TerminalState", "success": true,
      "inTransitions": [5005] }
  ],
  "transitions": [
    { "id": 5005, "inState": 5003, "outState": 5004,
      "condition": "ConditionRepository.cnd#3002" }
  ]
}
```

Note the state vocabulary: normal states are `"type": "State"`; success and
failure states are both `"type": "TerminalState"`, distinguished by the
`"success"` boolean.

### 3.3 The behaviour

A behaviour is ALICA's unit of domain-specific code. The engine knows only when
to start it, tick it, and stop it — everything inside is an opaque black box
that may report success or failure.

```cpp
// src/CountUp.cpp
void CountUp::run()
{
    alica::LockedBlackboardRW bb(*getGlobalBlackboard());
    int64_t next = bb.get<int64_t>(COUNTER_KEY) + 1;
    bb.set(COUNTER_KEY, next);
    std::cout << "[CountUp] agent " << getOwnId() << " counter = " << next
              << " (team size " << getTeamManager().getTeamSize() << ")" << std::endl;

    if (next >= TARGET) {
        setSuccess();   // this is what lets the plan progress
    }
}
```

Override `initialiseParameters()` for setup, `run()` for the tick, and
`onTermination()` for teardown; all three run on the behaviour's own thread.

Two blackboard traps: integers are **promoted to `int64_t`** on `set`, so you
must `get<int64_t>` (not `get<int>`); and `set` takes `T&&`, so an explicit
`set<int64_t>(key, lvalue)` fails to compile — let the template deduce.

### 3.4 Wiring ids to code

The engine turns model ids into C++ objects through six creator interfaces. In
production `alica_dynamic_loading` implements them by pulling symbols out of
shared libraries with `boost::dll`, keyed on the `libraryName` and
`implementationName` fields in the model files. Doing it by hand instead keeps
the example Boost-free and makes the mapping obvious:

```cpp
// src/Creators.h
class BehaviourCreator : public alica::IBehaviourCreator
{
public:
    std::unique_ptr<alica::BasicBehaviour> createBehaviour(
            int64_t behaviourId, alica::BehaviourContext& context) override
    {
        switch (behaviourId) {
        case 4001: return CountUp::create(context);
        default:   throw std::runtime_error("Unknown behaviour id");
        }
    }
};

class TransitionConditionCreator : public alica::ITransitionConditionCreator
{
public:
    alica::TransitionConditionCallback createConditions(
            int64_t conditionId, alica::TransitionConditionContext& context) override
    {
        switch (conditionId) {
        case 3002:   // CountReached
            return [](const alica::Blackboard* input,
                      const alica::RunningPlan* rp,
                      const alica::Blackboard* globalBlackboard) {
                alica::LockedBlackboardRO bb(*globalBlackboard);
                return bb.get<int64_t>(CountUp::COUNTER_KEY) >= CountUp::TARGET;
            };
        default: throw std::runtime_error("Unknown condition id");
        }
    }
};
```

All six creators must be supplied even when unused. The shortcuts:

- **Plans** — `return alica::BasicPlan::create(context);` unless you want
  plan-level `onInit`/`run`/`onTerminate` hooks.
- **Utility functions** — `return alica::BasicUtilityFunction::create(context);`
  gives you the default, which scores assignments purely by the role/task
  priorities in `Roleset.rst`.
- **Conditions** (plan pre/runtime) and **constraints** — only called if your
  plans actually declare them; can return `nullptr` otherwise.
- **Transition conditions** named `DefaultCondition` never reach your creator;
  the engine handles that name internally (and it always returns `false`).

### 3.5 Starting an agent

```cpp
// src/main.cpp — the five steps
const std::vector<std::string> configPaths{configPath};   // NOT {configPath} inline:
                                                          // that picks the deprecated overload
alica::AlicaContextParams params(agentName, configPaths,
        "Roleset", "MinimalMaster", /*stepEngine=*/false, agentId);

auto ac = std::make_unique<alica::AlicaContext>(params);

// 2. platform pieces — swap for the ROS proxies to go distributed
ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
ac->setTimerFactory<alica::AlicaSystemTimerFactory>();
ac->setLogger<alica::AlicaDefaultLogger>();   // static: once per process

// 3. seed the global blackboard before init
alica::LockedBlackboardRW(*ac->getGlobalBlackboardShared())
        .set<int64_t>(minimal::CountUp::COUNTER_KEY, 0);

// 4. hand over the factories and start
alica::AlicaCreators creators(
        std::make_unique<minimal::ConditionCreator>(),
        std::make_unique<minimal::UtilityFunctionCreator>(),
        std::make_unique<minimal::ConstraintCreator>(),
        std::make_unique<minimal::BehaviourCreator>(),
        std::make_unique<minimal::PlanCreator>(),
        std::make_unique<minimal::TransitionConditionCreator>());
ac->init(std::move(creators));

// 5. the engine ticks on its own thread; later:
ac->terminate();
```

`AlicaContext` is the whole public API surface — one object per agent, holding
clock, communicator, solvers, timers, tracing and the global blackboard.
`stepEngine=true` instead gives you a manually-stepped engine, which is how the
test suite gets deterministic behaviour.

### 3.6 CMake

Three packages is all a non-ROS agent needs:

```cmake
find_package(alica_solver_interface REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_dummy_proxy REQUIRED)

add_executable(minimal_alica src/main.cpp src/CountUp.cpp)
target_link_libraries(minimal_alica PRIVATE
  alica_engine alica_dummy_proxy alica_solver_interface)
```

### 3.7 Verified output

```
started agentA (id 1000)
started agentB (id 1001)
[CountUp] started
[CountUp] agent 1000 counter = 1 (team size 1)
[CountUp] agent 1001 counter = 1 (team size 2)
[CountUp] agent 1000 counter = 2 (team size 2)
...
[CountUp] agent 1000 counter = 6 (team size 2)
[CountUp] stopped
agentA final counter = 6
agentB final counter = 6
```

Team size climbing 1 → 2 is auto-discovery working: `agentA` starts alone and
picks up `agentB` on its next broadcast.

### 3.8 Where to go next

To grow this into something real: replace `AlicaDummyCommunication` with
`alicaRosProxy::AlicaRosCommunication` for cross-process agents; add a second
entrypoint with a different task to see genuine task *allocation* rather than
just shared cardinality; add a runtime condition with variables and quantifiers
plus `CGSolver` to get constraint-solved numeric outputs. The turtlesim sample
does all three and is the natural next read.

---

## 4. Full feature set

### 4.1 The language — Propositional ALICA

The core modelling vocabulary. ALICA is explicit that only four concepts hold
domain-specific code (behaviours, conditions, utility functions, constraint
descriptions); everything else is domain-independent structure the engine
reasons about.

| Concept | What it is |
| --- | --- |
| **Behaviour** | An atomic activity that can succeed or fail — the leaf of the hierarchy, and where your C++ lives. Configurable tick frequency, optional event-driven mode, optional initial deferral. |
| **Finite-state machine** | States connected by directed, precondition-guarded transitions. Note the inverted semantics vs. textbook FSMs: **states say what to do, transitions fire when goals are achieved.** |
| **State** | Normal states hold any number of behaviours, plans and plantypes (executed concurrently). Terminal states are empty and split into success and failure. |
| **Transition** | Directed, precondition-guarded, cannot self-loop; at most one per ordered pair; none leave terminal states. |
| **Conditions** | Three kinds: **pre** (must hold to start/pass), **runtime** (must hold to start *and* continue — continuously re-checked, aborts on failure), **post** (expected to hold afterwards; declarative only, **not evaluated at runtime**). |
| **Entrypoint** | Marks an FSM's initial state and its **cardinality** — min and max agents. Below minimum, an agent won't start at all. Also carries the task. |
| **Task** | An abstract label for a kind of activity (`Transport`, `Inspect`, `Leader`). Annotates entrypoints; the unit the engine allocates. |
| **Plan** | The central concept: a set of task-annotated FSMs working toward a shared goal, plus pre/runtime conditions, variables and a utility function. |
| **Plantype** | A set of alternative plans for the same goal. At runtime the engine picks whichever scores highest — plan selection by utility. |
| **Plan tree** | Plans and plantypes nest inside states. A **DAG at design time** (elements are reused), interpreted as a **tree at runtime** (each occurrence is a distinct instance that does not share progress with its siblings). |
| **Synchronisation** | Forces a group of agents across a set of transitions *simultaneously*, via a three-way handshake establishing mutual belief. Joint-intentions semantics — for things like two robots lifting a table together. |
| **Utility function** | Weighted sum `U(B) = w₀·pri(B) + w₁·sim(B) + Σ wᵢ·fᵢ(B)`. `pri` scores role/task fit; `sim` biases toward the *current* assignment to stop agents churning between equally-good options. Custom summands return a value in [0,1] or negative to forbid. |
| **Role** | Abstracts agents from tasks via **capabilities** (which may be abilities or properties). A role requires capabilities; an agent claims them until self-monitoring says otherwise. Each role declares per-task priorities; **negative priority forbids** the assignment outright. |

**Success semantics** (worth internalising, it surprises people): a plan
succeeds when *all* its success-required FSMs succeed; an FSM succeeds when the
number of agents reaching its success state ≥ the entrypoint's minimum
cardinality (or ≥ 1 if that minimum is 0). Consequences: an FSM marked
success-required with no success state can **never** succeed, and a plan with no
success-required FSM **always** succeeds. Failure is simpler — any agent
reaching any failure state fails the plan.

### 4.2 The language — General ALICA

Extensions that exist to make programs reusable rather than hard-coded. The
motivating question in the docs is "how do you specify *which* object to lift?"
without either hard-coding it or hiding it inside behaviour internals.

| Concept | What it is |
| --- | --- |
| **Configuration** | Key-value pairs attached to a behaviour/plan/plantype **in the context of a state**. Lets one `Drive` behaviour serve many destinations — and lets the same plan appear twice in one state with different configs (e.g. left arm, right arm). |
| **Variables** | **Free** variables belong to a plan/plantype/behaviour and always exist. **Quantified** (agent) variables are created per-agent by a quantifier over a scope (plan, FSM or state) and are **global** — the same name in a different context means the same variable. Only the universal agent quantifier is implemented in practice. |
| **Variable bindings** | Bind a plan's variable to a sub-variable further down the hierarchy, transitively. This is how constraints from different levels combine on one variable. Free variables only. |
| **Constraints** | Attached to pre- and runtime conditions (never postconditions), restricting variable ranges. **Weakly guarded**: solutions are asserted only if the guarding condition holds — deliberately not proving satisfiability, since that can be intractable. |
| **Blackboard** | Typed key-value store for passing data. Per-plan/behaviour blackboards with declared blueprints, input/output **key mappings** between parent and child, optional inheritance, plus a free-form **global blackboard** per agent. |
| **Placeholders** | `.plh` files declare a slot in the plan tree whose concrete implementation is chosen at startup via a JSON `placeholderMapping` passed to `AlicaContextParams`. Late binding for plan structure. |

### 4.3 Engine and runtime

| Feature | Detail |
| --- | --- |
| **Task allocation** | Distributed search for the best assignment of agents to entrypoints, scored by utility. `PlanSelector`, `TaskAssignmentProblem`, `PartialAssignment` with a pooled allocator for speed. |
| **Conflict resolution** | Agents can reach different conclusions. `AuthorityManager` + `CycleManager` detect assignment cycles and impose authority, with exponential backoff (`CycleDetection` in `Alica.yaml`). |
| **Role assignment** | `StaticRoleAssignment` matches capabilities to roles; reassigns when self-monitoring reports a lost capability. |
| **Team management** | Auto-discovery via announcements/queries, timeout-based liveness, per-agent plan-tree beliefs (`TeamObserver`, `SimplePlanTree`), and a team blacklist. |
| **Transition synchronisation** | Three-way handshake implementing the joint-intention semantics above. |
| **Variable synchronisation** | Solver results propagated across the team with TTLs and a merging threshold, so agents converge on shared numeric values. |
| **Rulebook** | The engine's operational core: applies transition/allocation/success/failure rules per tick, bounded by `MaxRuleApplications`. |
| **Failure handling** | Optional automatic handling (`AutoFailureHandling`): re-plan a failed plan, propagate failure upward, retry. |
| **Scheduling** | Each behaviour and plan runs at its own configured frequency, on its own thread, with an optional initial deferral. |
| **Runtime reconfiguration** | `setOption("Alica.CycleDetection.Enabled", …)` re-reads config live; components subscribe via `ConfigChangeListener`. |
| **Pluggable clock** | System clock or ROS clock (`AlicaClock`), so simulation time works. |
| **Pluggable communication** | `IAlicaCommunication`: in-process dummy, ROS 1, ROS 2 — or your own transport. |
| **Pluggable timers** | `IAlicaTimerFactory`, separately settable for the engine loop and for behaviours; a custom executor can be supplied (added in `dfc62ec7`). |
| **Pluggable logging** | `IAlicaLogger` with verbosity levels; default and ROS implementations. One logger per process (it is static). |
| **Tracing** | `IAlicaTrace`/`IAlicaTraceFactory` spans for behaviours and plans, with parent-context propagation — Jaeger-style distributed tracing of plan execution. Dummy and real backends. |
| **Event logging** | Optional structured event log to disk (`EventLogging` in `Alica.yaml`). |
| **Step mode** | `stepEngine=true` makes the engine advance only when told, which is what makes the test suite deterministic. |
| **Test harness** | `alica_test_utility` provides `TestContext` with `isStateActive(plan, state)`, `getActiveBehaviour(name)`, `getActivePlan(name)`, forced transitions via `setTransitionCond`, and fully-qualified-name lookup into the running plan tree. |

### 4.4 Solvers

Solvers are the only mechanism that assigns values to ALICA variables. The
engine defines `ISolver`/`ProblemDescriptor`/`Query`; implementations plug in via
`ac->addSolver<T>()`.

| Solver | What it does | State |
| --- | --- | --- |
| **Simple Solver** (`alica_simple_solver`) | Assigns values directly with no search; uses the blackboard to share values across the team. | Actively maintained |
| **CGSolver** (`supplementary/constraintsolver`) | Continuous nonlinear CSPs over the reals, gradient-based, on a C++ port of AutoDiff; includes an SMT extension from a 2013 paper. This is what places the turtles on their circle. | Actively maintained |
| **ASPSolver** | Answer-set programming via Clingo. No variable synchronisation. | External, unmaintained |
| **CACE** | Distributed tuple-space consensus rather than a solver proper. | External, ~6 years stale |

`supplementary/autodiff` is the reverse-mode automatic-differentiation library
CGSolver is built on; it is usable standalone and has 20 of its own tests.

### 4.5 Standard library

`alica_standard_library` ships reusable, domain-independent building blocks so
common plans need no custom C++ at all. Everything is exported via
`BOOST_DLL_ALIAS`, so you reference it from a model file by setting
`libraryName` to `alica_standard_library` — no code on your side.

- **Conditions** — success/failure aggregation over children (`AnyChildSuccess`,
  `AllChildSuccess`, `AnyChildFailure`, `AllChildFailure`, `IsChildSuccess`,
  `IsChildFailure`), constants (`AlwaysTrueCondition`, `AlwaysFalseCondition`),
  and blackboard comparisons (`IsEqual`, `IsNotEqual`, `IsGreaterThan`,
  `IsLessThan`, `IsGreaterThanOrEqual`, `IsLessThanOrEqual`).
- **Plans** — `UntracedPlan` and `TracedPlan` as ready-made plan
  implementations, plus `PopulateBlackboard`.
- **Behaviours** — `GenerateRandom`.

The child-success conditions are what let you build sequences and parallel
blocks purely in the model. Note that `TriggerFromInputCond` — the condition
`TestContext::setTransitionCond()` drives — lives in `alica_tests`, not here.

---

## 5. What is the "ALICA planner"?

Short answer: **there is no component called the "planner."** The string
`planner` does not appear anywhere in this repository. What people mean by it is
almost always the **ALICA Plan Designer** — and it is worth being precise,
because the distinction matters for what you can expect the framework to do.

### 5.1 What the Plan Designer actually is

A **web-based graphical editor** for authoring ALICA programs. It is the tool
that produces the `.pml`/`.beh`/`.cnd`/`.tsk`/`.rst` files described in
section 3 — you drag out states, draw transitions, define entrypoints with
cardinalities, attach conditions, and declare tasks and roles, instead of
hand-writing JSON and inventing ids.

It lives in `supplementary/alica_designer_runtime/` and is a full application
stack, not a library: a Django/Python backend, a JavaScript frontend, PostgreSQL
and Redis, run via Docker Compose.

```bash
cd supplementary/alica_designer_runtime
./run_designer.sh start      # also: reset (wipe DB), update (pull images)
# then open http://localhost:3030/
```

Configuration is in `config.env` — `BACKEND_PORT` (9000), the frontend on 3030,
GitHub OAuth credentials, and `NATIVE_MODE` plus `NATIVE_IMPORT_EXPORT_PATH` to
enable reading and writing plan files straight to your filesystem instead of
through the browser. It can import and export projects via filesystem, ZIP or
Git, and there is a `generate.sh` code-generation step that scaffolds C++ stubs
for the behaviours and conditions you declared (it needs `git-lfs`).

Docs live in `supplementary/alica_designer_runtime/doc/`. An older desktop
JavaFX edition exists separately as `alica-plan-designer-fx`.

The framework documentation describes ALICA as three parts: **the language, the
engine, and the plan designer.** The designer is that third part — an authoring
tool, entirely design-time. Nothing in it runs on a robot.

### 5.2 What ALICA deliberately does *not* do

ALICA is **not an AI planner** in the classical sense. It does not take a goal
and a set of action models and *synthesise* a plan — there is no PDDL, no
STRIPS, no HTN decomposition, no search over world states. You write the plans;
the engine *executes* and *allocates* them.

The framework's own documentation is explicit about this. From
`docs/articles/conditions.md`, on why postconditions exist at all:

> This, for example, allows the application of classic planning algorithms to
> the ALICA language. **Since planning is currently not supported by the ALICA
> Framework, postconditions are not used at runtime.**

So postconditions are a hook for a planner that was never built. If you need
goal-directed plan synthesis, you would layer a planner *on top* of ALICA and
have it emit or select plans.

### 5.3 So what does ALICA compute at runtime?

Plenty — just not plan synthesis. Three things get *solved* while your agents
run, which is where the "planner" intuition comes from:

1. **Task allocation** — a distributed combinatorial search for the best
   assignment of agents to entrypoints, maximising utility subject to
   cardinality constraints. This is genuine optimisation, re-run continuously as
   the team and situation change.
2. **Plan selection** — choosing the best plan from a plantype by comparing
   utility functions. A form of runtime decision-making over pre-authored
   alternatives.
3. **Constraint solving** — CGSolver computing numeric values (positions,
   velocities) for variables under nonlinear constraints.

A reasonable one-line summary: ALICA is a **plan executor with distributed task
allocation**, authored through a graphical designer, not a planner.

---

## 6. ALICA vs. off-the-shelf alternatives

### 6.1 Where ALICA sits

The honest framing: most popular alternatives are **single-robot behaviour
coordination** tools. ALICA's distinguishing claim is **multi-agent** — that N
agents running the same program will negotiate a consistent division of labour
without a central coordinator, and re-negotiate when an agent joins, leaves or
breaks. If you only ever have one robot, most of ALICA is overhead you won't use.

| | ALICA | Behaviour Trees (BehaviorTree.CPP / Nav2) | SMACH / FlexBE | ROS 2 Executors + custom code | PDDL/HTN planners (POPF, PlanSys2) | Multi-agent auction frameworks |
| --- | --- | --- | --- | --- | --- | --- |
| **Primary target** | Teams of agents | One robot | One robot | One robot | One or more, offline | Teams |
| **Coordination model** | Distributed task allocation by utility, no central node | None built in | None built in | Hand-rolled | Centralised solve | Auctions / market |
| **Authoring** | Graphical designer → JSON model | XML (+ Groot editor) | Python / web editor | Code | PDDL domain files | Code |
| **Plan synthesis** | No | No | No | No | **Yes** | No |
| **Reactivity** | Continuous re-evaluation, runtime conditions abort | Tick-based, reactive | Transition-based | Whatever you write | Weak — replanning is expensive | Varies |
| **Failure handling** | Built in, configurable, propagates hierarchically | Fallback/decorator nodes | Explicit outcomes | Manual | Replan | Re-auction |
| **Language** | C++17 only | C++ (Python bindings) | Python | C++ / Python | C++ / Python | Varies |
| **Ecosystem size** | Very small | Large | Medium (SMACH ageing) | Enormous | Small–medium | Fragmented, mostly academic |

### 6.2 Pros

**Real multi-agent coordination, decentralised.** The headline feature, and
genuinely uncommon in open source. Task allocation, conflict resolution via
authority and cycle detection, transition synchronisation with joint-intention
semantics, agent discovery and death handling, variable synchronisation across
the team — all built in and all tested. Reproducing even a fraction of this on
top of behaviour trees is a serious project.

**Cardinality as a first-class constraint.** "This FSM needs 1–2 agents, and
below one nobody starts" is declarative. Expressing that in a BT means writing
your own coordination protocol.

**Utility-driven allocation with anti-churn built in.** The similarity summand
existing at all shows real deployment experience: without it, agents thrash
between equally-scored assignments. Negative role/task priorities give you hard
prohibitions alongside soft preferences.

**Clean domain-independence boundary.** Exactly four concepts hold your code;
everything else is structure the engine reasons about. This is well-designed and
consistently enforced, and it is why the same engine served robotic soccer,
space exploration, autonomous driving and warehouse automation.

**Genuinely pluggable platform layer.** Clock, communication, timers, logging,
tracing and solvers are all interfaces. This is why the core builds and its full
test suite passes with **no ROS at all** — verified above. Being able to test
multi-agent logic in-process with a dummy communicator, deterministically via
step mode, is a real advantage over frameworks welded to a middleware.

**Hierarchical composition that scales.** Plans nest into plans and plantypes,
with a locality principle on utility functions that keeps runtime evaluation
tractable as programs grow.

**Production track record.** Rapyuta Robotics has run this in customer
warehouses since 2019, including multi-floor autonomous forklifts. Not a
research prototype.

**Serious test coverage.** 123 tests across four suites, covering the hard parts
— authority conflicts, sync transitions, agent death, blackboard inheritance,
failure propagation.

### 6.3 Cons

**Tiny ecosystem, and that is the dominant practical cost.** BehaviorTree.CPP
has broad adoption, active maintenance, tutorials, Stack Overflow answers and
Nav2 integration. ALICA has one primary corporate user, a handful of maintainers
and a documentation site with sections that read "to be done" — including
`task_allocation.md`, the article describing its single most important
algorithm. Expect to read source to answer questions.

**Steep, unusual conceptual load.** Plans, plantypes, entrypoints, tasks, roles,
capabilities, utility functions, quantified vs. free variables, variable
bindings, weak guards, configurations, placeholders, blackboard key mappings.
The FSM semantics are inverted from the textbook. Nothing here is arbitrary, but
the ramp is long — much longer than "a tree of nodes that tick."

**Model files are not really hand-authorable at scale.** They are JSON with
64-bit integer ids and cross-file `file#id` references. The minimal example
proves hand-writing is *possible*, but the intended workflow needs the Plan
Designer — and that is a Docker Compose stack with PostgreSQL, Redis, a Django
backend and OAuth config. Compare a BT XML file you can edit in any text editor.
Diffing and code-reviewing generated plan JSON is unpleasant, and ids make merge
conflicts nasty.

**Rough packaging.** Documented in section 1.5: two packages silently override
`CMAKE_BUILD_TYPE` to Debug so you cannot get an optimised engine without
editing CMake; the yaml-cpp linkage depends on a Debian patch and breaks
confusingly against upstream yaml-cpp; there is no top-level `CMakeLists.txt`,
so you sequence nine packages yourself; `LD_LIBRARY_PATH` is read directly at
runtime and throws if unset. None is fatal — all are friction, and they signal a
project built around one organisation's CI rather than for outside consumption.

**No planning, and postconditions are dead weight.** If you want goal-directed
synthesis, ALICA is the wrong layer; you would pair it with PlanSys2 or similar.
Postconditions exist in the language but are explicitly unused at runtime — a
modelling feature that looks functional and is not.

**C++17 only.** No Python bindings. The docs mention Java and Python engines as
"currently work-in-progress," which given the repository's activity should be
read as aspirational. For teams prototyping in Python this alone may decide it.

**Dynamic loading adds real operational complexity.** The production path
resolves behaviours through `boost::dll` by scanning `LD_LIBRARY_PATH` for
`lib<libraryName>.so` and importing a symbol named after `implementationName`.
Powerful — you can swap implementations without recompiling the engine — but
failures surface as runtime `DynamicLoadingException`s rather than link errors,
and the error messages are the only debugging aid.

**Some pieces are stale or external.** Two of the four documented solvers are
unmaintained and live in other repositories. Tracing and the designer runtime
depend on images from a private registry (`quay.io/rapyuta/...`).

### 6.4 Choosing

**Choose ALICA when** you have a genuine team of agents that must divide work
among themselves; when team composition changes at runtime and the system must
adapt; when you need synchronised joint actions; when heterogeneous agents
should take different roles by capability; or when coordination must survive the
loss of any single agent, including a coordinator.

**Choose something else when** you have one robot (use BehaviorTree.CPP —
smaller, better documented, larger community); when you need goal-directed
planning (use a PDDL/HTN planner, possibly *above* ALICA); when your team is
Python-first; when a central coordinator is acceptable and simpler, which for
small fixed fleets it often is; or when ecosystem maturity and hiring matter
more than the coordination features.

**A fair summary:** ALICA solves a harder problem than its popular alternatives,
and solves it with a coherent design and real deployment history. You pay for
that in learning curve, tooling friction and ecosystem thinness. If distributed
multi-agent coordination is your actual problem, that trade is often worth it —
and there is little else in open source that does this. If it is not your
problem, the cost is hard to justify.

---

## Appendix: reference

### Repository layout

| Path | Contents |
| --- | --- |
| `alica_engine/` | the engine: model, rulebook, plan selector, team manager, blackboard |
| `alica_solver_interface/` | solver-agnostic interfaces and interval arithmetic |
| `alica_simple_solver/` | trivial value-assignment solver |
| `alica_dynamic_loading/` | `boost::dll`-based creators |
| `alica_standard_library/` | reusable plans and conditions |
| `alica_dummy_proxy/` | in-process communicator for tests and single-process teams |
| `alica_test_utility/` | `TestContext` and test helpers |
| `alica_tests/` | the 92-test main suite plus its model files in `etc/` |
| `examples/minimal/` | the minimal non-ROS example from section 3 |
| `supplementary/autodiff/` | automatic differentiation |
| `supplementary/constraintsolver/` | CGSolver |
| `supplementary/alica_ros1/`, `alica_ros2/` | ROS proxies and turtlesim demos |
| `supplementary/alica_turtlesim/` | shared turtlesim plans and behaviours |
| `supplementary/alica_designer_runtime/` | the Plan Designer stack |
| `supplementary/alica_tracing/`, `alica_dummy_tracing/` | tracing backends |
| `docs/articles/` | the language documentation, ~5-10 min per article |
| `cmake_flags/cflags.cmake` | shared compile flags |

### Useful links

- GitHub Pages: <https://rapyuta-robotics.github.io/alica/>
- ROSWorld 2021 talk: <https://vimeo.com/649645073>
- Plan Designer (desktop edition): <https://github.com/rapyuta-robotics/alica-plan-designer-fx>
- Start reading at `docs/README.md`, then `docs/articles/domain-independence.md`
