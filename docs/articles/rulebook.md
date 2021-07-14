# Rulebook

*The following explanation is partially based the PhD Thesis of Stephan Opfer [1]*

The Rulebook is the central part of the operational semantics of the ALICA runtime engine. Given a runtime representation of a plan of the ALICA program, the Rulebook decides about the next step in this plan, according to its rules and the priorities among the rules. The following paragraphs list the rules with decreasing priority and give a summary of the meaning of each rule.

## Progress Rules

**Initialisation** The Initialisation rule is triggered if the agent is starting to execute an ALICA program. As a result, the agent creates the runtime representation of the ALICA program, a. k. a. Running Plan. Further the agent is occupying the initial state of the top-level plan, and it is remembered that the [task allocation](task_allocation.md) for the initial state is required to be triggered.

**Allocation** Whenever it is deemed necessary to make an initial task allocation within the context of state, e. g., because an agent just entered the state by following a transition, the Allocation rule starts the [task allocation](task_allocation.md), in order to recursively assign tasks within the state and all reachable child plans.

**Dynamic Allocation** A crucial role for a task allocation is its utility according to the current situation. The Dynamic Allocation rule recurrently reevaluates the utility of all task allocations and triggers a new task allocation (via the Allocation rule), if the agent believes that another allocation is of higher utility. In difference to the Allocation rule, the Dynamic Allocation rule only evaluates the current level and is not recursively doing a complete task allocation.

**Authority Override** It is unavoidable that there emerge inconsistencies between the task allocations calculated by different agents. If this happens continuously for a configurable amount of times, the Authority Override rule allows one agent of the team to override  the task allocations of all other agents that are in conflict with it.

**Transition** The transition rule is continuously checked for each currently executed plan. The preconditions of all outgoing transitions of the current active state are checked. If the precondition holds, the other agents, that also occupied the current active state are expected to move along the transition, too. This assumption reduces inconsistent runtime representations of the ALICA program within the team, because the runtime engine makes optimistic assumptions about the progress of other teammates, instead of relying on potentially delayed communication.

**Synchronised Transition** The Synchronised Transition rule initialises the synchronisation of synchronised transitions via the Sync Module of the ALICA runtime engine. In case the Sync Module established a mutual belief to pass a synchronised transition, the Synchronised Transition rule applies the transition over the synchronised transition by stopping the execution of every plan, plantype, or behaviour in the current state, moving the agent and its teammates to the state over the transition and demands a task allocation in this new state, just like the Transition rule would do.

## Repair Rules

All rules described so far are denoted as operational rules, have higher precedence than the following rules, and are sufficient for the execution of an ALICA program. The following rules are denoted as repair rules. As the name implies, the repair rules handle faulty execution of plans

The general repair strategy implemented in the ALICA runtime engine is to restart the behaviour, plan, or task allocation up to a configurable number of retries and, in case it was not possible to repair runtime representation on the current plan level, to propagate the failure up the plan hierarchy. 

**Plan Abort** The Plan Abort rule recognises that a plan should be stopped from execution. This is the case, if a failure state is reached, the current task assignment is invalid, the runtime condition of the plan does not hold anymore. However, it does not stop the execution, it only increases the failure count for the corresponding plan. As a result, the following rules will react to the increased failure count.

**Plan Redo** If the failure count of a plan is 1, all agents are moved back to their initial state and the task allocation for the initial state is requested via the Allocation rule. Like the name says, the plan is restarted.

**Plan Replace** If the failure count of a plan is 2, the task allocation for the state at the parent plan is requested for all plans in that state.

**Plan Propagate** If the failure count of a plan is 3 or larger, the failure count of the parent plan is increased.

**Top Fail** The Top Fail rule restarts the top-level plan by moving all agents into the initial state. Essentially, it is like the Initialisation rule.

[1] Stephan Opfer. '[Symbolic Represenation of Dynamic Knowledge for Robotic Teams](https://kobra.uni-kassel.de/handle/123456789/12830)'. PhD Thesis. Wilhelmshöher Allee 73, D-34117 Kassel: University of Kassel, March 2021.

**NAV** *prev: [Domain-Independence](domain-independence.md)*  *top: [Overview](../README.md)*

