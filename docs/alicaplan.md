# alicaplan: plans as short YAML

`alicaplan` (in `tools/alicaplan`) turns a short YAML description of plans, behaviours, tasks,
conditions and roles into the JSON files the engine loads, and checks engine JSON for broken
wiring. It is a Python tool; install it with `uv tool install ./tools/alicaplan`.

| Command                          | What it does                                                                                                                                                                                                   |
| -------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `alicaplan compile SRC -o OUT`   | Compiles every `*.alica.yaml` under `SRC` into `OUT`, runs the checks, and writes nothing if a check fails (`--force` writes anyway). `--clean` deletes model files in `OUT` that the compile did not produce. |
| `alicaplan check DIR...`         | Checks all `.pml .beh .pty .plh .tsk .cnd .rst` files under the folders. Exits 1 on errors (`--strict`: on warnings too).                                                                                      |
| `alicaplan decompile DIR -o SRC` | Turns engine JSON back into `*.alica.yaml` plus `*.layout.yaml`, keeping every id. `--no-ids` drops them, so the next compile derives new ones from names.                                                     |

`compile` after `decompile` reproduces the original files byte for byte; the tests do this for
all of `alica_tests/etc` and `supplementary/alica_turtlesim/etc`.

`alica_load_plans MASTER_PLAN ROLE_SET DIR...` (built with `alica_engine`, installed into
`install/bin`) parses a plan tree and a role set with the engine's own model manager and exits
non-zero if the engine rejects them. Pass `--placeholder-mapping FILE` for trees with
placeholders.

## Files

A source folder holds any number of modules named `*.alica.yaml`. Each element of a module
becomes one engine file in the matching folder of the output: `src/plans/attack.alica.yaml`
writes `out/plans/<Name>.pml`, `out/plans/<Behaviour>.beh` and so on. The engine finds files by
name in all subfolders, so names must be unique per file type.

A module may have these top-level keys:

| Key                                                  | Becomes                                                                                        |
| ---------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `library`                                            | Default `libraryName` for the plans, behaviours and conditions of the module.                  |
| `plans`, `behaviours`, `plantypes`, `placeholders`   | Mapping of name to definition; one file each.                                                  |
| `task_repository`, `condition_repository`, `roleset` | One repository file. `name` defaults to `TaskRepository`, `ConditionRepository` and `RoleSet`. |

Positions live next to the module in `<module>.layout.yaml`, so the plan source stays free of
coordinates. Elements without a position get a simple grid position.

```yaml
Offense: # plan name
  entryPoints: { Attack: [100, 100] } # keyed by task
  states: { GetBall: [300, 100], Scored: [520, 100] }
  synchronisations: { pass: [400, 20] }
  transitions: { GetBall -> PassToMate: [[350, 60], [380, 60]] } # bend points
```

## Example

```yaml
library: football_demo

task_repository:
  tasks: [Attack, Support]

condition_repository:
  conditions:
    PassReady: {}
    BallLost: {}
    HasBall:
      blackboard: { ball_owner: int64, me: int64 }
    AnyChildSuccess: { library: alica_standard_library }

behaviours:
  Dribble:
    blackboard: { target_x: double }
  Kick: {}

plans:
  Offense:
    blackboard: { goal_x: double, ball_owner: int64, me: int64 }
    entryPoints:
      - { task: Attack, state: GetBall, min: 1, max: 1 }
      - { task: Support, state: GetOpen }
    states:
      GetBall:
        - Dribble:
            in: { target_x: goal_x }
      Pass: [Kick]
      GetOpen: [Kick]
      Scored: { terminal: success }
      failure: null
    transitions:
      - GetBall -> Pass: PassReady
      - GetOpen -> Scored:
          condition: HasBall
          in: { ball_owner: ball_owner, me: me }
      - Pass -> failure: BallLost
    synchronisations:
      pass:
        transitions: [GetBall -> Pass, GetOpen -> Scored]

roleset:
  roles:
    Striker:
      priorities: { Attack: 0.9, Support: 0.5 }
```

`tools/alicaplan/examples/football` has a larger example that compiles clean and loads in the
engine.

## Reference

Every element also accepts `id` (otherwise derived from its name, see below) and `comment`.
Defaults are given in brackets; a key left out takes its default.

### References

- A plan, behaviour, plan type or placeholder is referenced by name. When two kinds share a
  name, add the extension: `Tackle.beh`, `Tackle.pml`.
- A task or condition is referenced by name, or `Repository/Name` if two repositories share it.
- Anything written as the engine's own `File.ext#id` is passed through unchanged.

### Plans

| Key                                                                                           | Engine field                                                                                                            |
| --------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| `master` [false]                                                                              | `masterPlan`                                                                                                            |
| `utilityThreshold` [0.1], `frequency` [0], `inheritBlackboard` [false], `isInterface` [false] | same name                                                                                                               |
| `library` [module], `implementation` [""]                                                     | `libraryName`, `implementationName`                                                                                     |
| `blackboard`                                                                                  | see Blackboards                                                                                                         |
| `pre`, `runtime` [none]                                                                       | `preCondition`, `runtimeCondition` (see Condition objects)                                                              |
| `variables` [[]]                                                                              | `variables`, written in engine form                                                                                     |
| `entryPoints`                                                                                 | list of `{task, state, min [0], max [inf], successRequired [false], isDynamic [false], name [""]}`                      |
| `states`                                                                                      | mapping of state name to definition                                                                                     |
| `transitions`                                                                                 | list of `From -> To: Condition` or `From -> To: {condition, in, out, name, pre}`                                        |
| `synchronisations`                                                                            | mapping of name to `{transitions: [From -> To, ...], syncTimeout [10000], talkTimeout [30], failOnSyncTimeout [false]}` |

A state is one of:

- a list of children: `GetBall: [Dribble, Look]`;
- a mapping `{children, name, variableBindings}`; `name` overrides the engine name;
- a terminal state: `{terminal: success}` or `{terminal: failure}`, optionally with `post` (a
  condition object). The key is its name: `Scored: {terminal: success}`.
- an unnamed terminal state: the key `success`, `failure`, `success.2`, ... with no body. The
  engine names them `Success`, `Failure`, `Success2`.

A child is a name, or `Name: {in, out, name}` to add key mappings. When two transitions join the
same pair of states, the second is `From -> To #2` in synchronisations and layout files.

### Key mappings

`in` and `out` are keyed by the key that receives the value:

```yaml
in: # child key: parent key, or a constant
  target_x: goal_x
  power: { value: 0.8 }
out: # parent key: child key
  scored: scored
```

On a transition, the condition is the child: `in` maps plan keys into the condition's
blackboard.

### Blackboards

A mapping of key to type, or to `{type, access, default, comment}`. Types are `bool`, `int64`,
`uint64`, `double`, `std::string` and `std::any`. `access` is `input`, `output` or `protected`;
it defaults to `protected` in plans and to `input` in behaviours, conditions and placeholders.

### Behaviours, plan types, placeholders

- Behaviour: `frequency` [30], `deferring` [0], `eventDriven` [false], `inheritBlackboard`,
  `isInterface`, `library`, `implementation`, `blackboard`, `pre`, `runtime`, `post`,
  `variables`.
- Plan type: `plans`, a list of plan names or `Plan: {activated [true], name}`; the annotated
  plan is named `Annotated<Plan>` by default. Also `variables`, `variableBindings`.
- Placeholder: `blackboard`.

### Repositories

```yaml
task_repository:   {name, tasks: [Attack, {Keep: {comment: goalkeeper}}]}
condition_repository: {name, conditions: {HasBall: {library, blackboard}}}
roleset:           {name, defaultPriority [0.5], defaultRoleSet [true],
                    roles: {Striker: {priorities: {Attack: 0.9}, characteristics}}}
```

### Condition objects

`pre`, `runtime` and `post` take `{name, conditionString, enabled [true], quantifiers, variables}`. Every transition gets one as its `preCondition`. A quantifier is `{scope, sorts, quantifierType [all], name}`; in a plan, its `scope` may be
written as `plan`, `state:<name>` or `ep:<task>`.

### Ids

Ids are derived from names: a SHA-256 of a path such as `pml:Offense/state:GetBall`, cut to 63
bits. Renaming an element changes its id; reordering does not. An explicit `id` always wins,
which `decompile` uses to keep the ids of existing plans that code depends on.

## Checks

`check` and `compile` report these. Errors make the command fail.

| Code                                                  | Severity | Meaning                                                                                                                                                                                             |
| ----------------------------------------------------- | -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `parse`, `schema`, `bad-id`                           | error    | A file is not valid JSON or an element has the wrong shape or no integer id.                                                                                                                        |
| `duplicate-id`, `duplicate-file`                      | error    | Two elements share an id, or two files share a name (the engine loads whichever it finds first).                                                                                                    |
| `ref-missing`, `ref-unresolved`, `ref-malformed`      | error    | A reference is unset, points at no element, or is not `File.ext#id`. A transition without a condition is reported here; the engine would crash evaluating it.                                       |
| `ref-wrong-file`, `ref-wrong-kind`                    | error    | The file part of a reference names another file than the one holding the id, or the id is the wrong kind of element.                                                                                |
| `back-ref`                                            | error    | Two sides of a link disagree: `parentPlan`, `entryPoint`, `inTransitions`/`outTransitions`, synchronisations, repository back-references.                                                           |
| `entry-point`                                         | error    | No entry points, invalid cardinality, two entry points on one state, or an entry point starting in a terminal state.                                                                                |
| `terminal-state`                                      | error    | A success or failure state has outgoing transitions or runs plans.                                                                                                                                  |
| `master-terminal`                                     | error    | A master plan has a success or failure state; reaching it ends the agent's plan execution.                                                                                                          |
| `key-missing`, `key-type`, `key-mapping`              | error    | A key mapping names a key that is not in the blackboard, maps between different types (`std::any` accepts any type), maps a constant that does not parse or into `std::any`, or maps one key twice. |
| `blackboard`, `duplicate-name`, `plantype`, `roleset` | error    | Unsupported blackboard type or access, a duplicated key or name, or a broken plan type or role.                                                                                                     |
| `unreachable`                                         | warning  | A state cannot be reached from any entry point.                                                                                                                                                     |
| `no-terminal`                                         | warning  | A non-master plan has no reachable success or failure state, so it can only end through its runtime condition.                                                                                      |
| `synchronisation`, `name-mismatch`                    | warning  | A synchronisation with fewer than two transitions; an element name that differs from its file name.                                                                                                 |

The known findings in the test fixtures are three master plans with success states and five
transitions without a condition in `OrderedSchedulingTestPlan.pml`; the tests pin them.
