# Plan Trees

**Note:** Before reading this article, you should already know what core concepts a [plan](./plans.md) can include: behaviours, states, transitions, conditions, entrypoints, tasks, etc...

In the ALICA language it is possible to create plan trees by adding plans and plantypes into states (not just behaviours). In fact you can insert an arbitrary number and combination of behaviours, plans, and plantypes into a state. Lets consider the following example with just one behaviour, plan, or plantype in each state to see how the plan tree hierarchy is build:

![serve_plan_tree_example](../images/serve_plan_tree_example.svg)

_Figure 2: Plan Tree for Domestic Service Robots_

The _Serve_ plan is the root of the plan hierarchy, depicted in Figure 2. In State _Z0_ and _Z2_ the two behaviours _Wait_ and _Charge_ are inserted, respectively, just like you have seen it before. However, State _Z1_ includes the _Assist_ plantype, which is composed of the three plans _Clean Up_, _Find Item_, and _Transport_. Since _Wait_, _Charge_, _Clean Up_, _Find Item_, and _Transport_ are located in a state of _Serve_, they are denoted as the children of _Serve_. This all is in line of a classic [tree](<https://en.wikipedia.org/wiki/Tree_(data_structure)>) data structure.

However, the _Clean Up_ plan itself again includes the _Transport_ and _Find Item_ plan, which its parent already included as part of the _Assist_ plantype. Therefore, the plan hierarchy shown in Figure 2 is not a plan tree, but a directed-acyclic graph (DAG) of plans with a single root plan as shown in Figure 3.

![dag_plan_hierarchy_example](../images/dag_plan_hierarchy_example.svg)

_Figure 3: Visualisation of the Serve Plan Hierarchy as Directed-Acyclic Graph_

The DAG in Figure 3 is the representation of an ALICA program at design time. Here, any change you make to, for example, the _Find Item_ plan will be affect all its occurrences in the DAG. At runtime, however, the DAG will be interpreted as a tree and each occurrence of _Find Item_ will be a different instance. This means, for example, that agents in different instances don't cooperate by sharing their progress in the FSMs and cannot contribute to the success of their instances together (see [Success Semantics of Plans](./plans.md)).

The annotations on the edges in Figure 3 state over which task, state, and plantype (optional) the child is connected to its parent. The edge between the _Serve_ and _Clean Up_ plan, for example, is annotated with (Serve, Z1, Assist), i.e. you reach the _Clean Up_ plan when you take on task _Serve_, progress to state Z1, and choose the _Clean Up_ plan from the plans in the _Assist_ plantype. How this choice is made is described in more details in the [Utility Functions](./utility_functions.md) and [Task Allocation](./task_allocation.md) articles.

## Summary

Adding plans and plantypes into states allows to create multiple levels in the hierarchy of ALICA programs. In total, there are three different types of child-relationships in ALICA programs:

- **Behaviours:** Plan -- (Task, State) --> Behaviour
- **Plans:** Plan -- (Task, State) --> Plan
- **Plantypes:** Plan -- (Task, State, Plantype) --> Plan

At design time an ALICA program is considered a DAG, because behaviours, plans, and plantypes can be reused multiple times in different locations. At runtime this DAG is interpreted as tree.

**NAV** _prev: [Roles](roles.md)_ _top: [Overview](../README.md)_
