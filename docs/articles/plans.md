# Plans

As prerequisite to understand this article about plans, we require you to read the following articles first:

* [Behaviours](behaviours.md)
* [Finite-State Machines](finite-state_machines.md)
* [Conditions](conditions.md)
* [Entrypoints](entrypoints.md)
* [Tasks](tasks.md)

Quick recap: Finite-state machines (FSM) place behaviours into their states and connect them with precondition-guarded transitions. Terminal states can be used to signal the outcome of a FSM. Entrypoints determine the cardinalities of FSMs and point to their initial state. And tasks abstractly annotate FSMs with what is done in this FSM.

Separating different goals into different FSMs makes sense from a [separation-of-concerns perspective](https://en.wikipedia.org/wiki/Separation_of_concerns). However, in many cases different goals contribute  to a common greater good. In such cases, FSMs need to be combined. Exactly this is the job of plans. Plans include the majority of all core concepts, which makes plans the central core concept of the ALICA language. 

Let's consider the following example:

![Plan Example: Cleaning Up](../images/clean_up_plan_example.svg)

*Figure 1: Plan for Cleaning up Items in a Household*

Figure 1 shows a plan for cleaning up your household. The plan is made up of two FSMs. The upper FSM lets agents bring back items where they belong until no item is left. The lower FSM makes agents search for items that need to be cleaned. Both FSMs are annotated with an entrypoint (blue dot) that provides the cardinalities and task of that FSM. Further, the upper entrypoint states that its FSM is required to be successful, in order for the plan to be successful (see [Success Semantics](#Success-Semantics)). Finally, according to the precondition, the plan is only started when it is messy.

As you can see by the different tasks, the two FSMs have a different purpose but together they work towards the common goal of cleaning up your household. In the Transport task FSM, agents need to carry things arround, while in the Inspect task FSM, agents need to identify items. This poses different requirements for the capabilities of the agents that are assigned to these FSMs. The problem of assigning agents to FSMs is solved by the ALICA Framework with the help of [tasks](tasks.md), [roles](roles.md), and [utility functions](utility_functions.md). Details can be found in the [task allocation article](task_allocation.md).

An open question now is: How do agents in the first FSM get to know about items found and identified by agents in the second FSM?

Obviously this part is about specifics of the household domain. Therefore, it would be fair to implement the domain-specific information sharing outside the scope of the ALICA program and only provide the agents with available domain-specific information via the domain-specific code of behaviours and conditions. However, the core concepts of the [General ALICA](../README.md#General-ALICA) part also provide features that help to address this issue.

## Success Semantics

For the progress in an ALICA program, it is important to know when a plan is successful or when it failed. 

to be done

**NAV** *prev: [Tasks](tasks.md)*  *top: [Overview](README.md)* *next: [Plantypes](plantypes.md)*