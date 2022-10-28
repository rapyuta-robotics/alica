# Tasks

In the ALICA language, the concept of a task describes an abstract kind of structured activity. In robotic soccer, for example, a typical task is to _attack_, in the domain of service robots, it is to _clean up_, and in warehouse automation it is to _transport_. All these tasks are rather abstract, hence there can be multiple ways to implement them. In robotic soccer you can attack by dribbling through the middle, playing a long pass towards the goal, or having a sequence of short passes. Each variant would be implemented by a different [finite-state machine](finite-state_machines.md) (FSM), but all FSMs would implement an _attack_ task.

Therefore, tasks annotate FSMs by being attached to their [entrypoints](entrypoints.md). FSMs that have the same task attached to their [entrypoints](entrypoints.md) are expected to be similar with respect to the actions that are executed in the FSMs. This allows the ALICA Engine to decide which agent should execute which FSM. More details about how the ALICA Framework solves the problem of assigning FSMs to agents are given in the [task allocation article](task_allocation.md).

**Remember:** A task is just an abstract way of annotating FSMs and should be reusable for multiple FSMs that do similar things.

**NAV** prev: [Entrypoints](entrypoints.md) _top: [Overview](../README.md)_ _next: [Plans](plans.md)_
