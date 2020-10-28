# Conditions

Conditions are just like behaviours, one of the four core concepts in the ALICA Framework, that encapsulate domain-specific code. Hence, the domain-independent ALICA Framework handles conditions as black-boxes. From the Frameworks' perspective, conditions can be true or false and the conditions must provide a method to determine their current truth value.

Further, the ALICA language distinguishes between three different kinds of conditions: pre-, runtime, and postconditions. A precondition must hold before something is done. The runtime condition must hold before and while something is done. And finally, the postcondition is only expected to hold after something has been done. 

Please note the different semantics between postconditions and the other two conditions. While pre- and runtime conditions control the progress of an ALICA program, the postconditions have no influence on that at all. Their simple purpose is to allow the designer of an ALICA program to express what outcome is expected from a certain element. This, for example, allows the application of classic planning algorithms to the ALICA language. Since planning is currently not supported by the ALICA Framework, postconditions are not used at runtime.

| Condition         | Context                                   | Semantics                                                    |
| ----------------- | ----------------------------------------- | ------------------------------------------------------------ |
| Precondition      | [Transition](finite-state_machines.md)    | A transition can only be passed, if the precondition holds.  |
|                   | [Behaviour](behaviours.md)                | A behaviour can only start, if the precondition holds.       |
|                   | [Plan](plans.md)                          | A plan can only start, if the precondition holds.            |
| Runtime Condition | [Behaviour](behaviours.md)                | A behaviour can only start and run, if the runtime condition holds. |
|                   | [Plan](plans.md)                          | A plan can only start and run, if the runtime condition holds. |
| Postcondition     | [Terminal State](finite-state_machine.md) | If an agent reaches a terminal state, the postcondition is expected to hold. |
|                   | [Behaviour](behaviours.md)                | If a behaviour is successful, the postcondition is expected to hold. |

*Table 1: Overview of Conditions with their Contexts and Semantics in those Contexts*

**NAV** prev: [Finite-State Machines](finite-state_machines.md) *top: [Domain-Independence](domain-independence.md)* *next: [Entrypoints](entrypoints.md)*

