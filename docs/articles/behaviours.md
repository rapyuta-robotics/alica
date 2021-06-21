# Behaviours

A behaviour is considered a low-level building block of an ALICA program. It represents an arbitrary atomic activity whose execution can either succeed or fail.  Driving to a given goal position, for example, is a typical behaviour. Please note that in general driving is by no means an atomic activity, but it is from the perspective of the ALICA language. The skill-set of an autonomous agent is typically reflected by the set of behaviours that  an agent can execute.

The execution of a behaviour is guarded by three [conditions](conditions.md), namely a pre-, runtime, and postcondition. All conditions can either be true or false. The precondition must hold when the activity is started. The runtime condition must hold when the activity is started and during its execution. And finally, the postcondition is expected to hold, when the activity succeeded.

In order to decide which behaviour is executed when, the ALICA language utilises the well-known concept of [finite-state machines](finite-state_machines.md).

**NAV** *prev: [Domain-Independence](domain-independence.md)*  *top: [Overview](../README.md)* *next: [Finite-State Machines](finite-state_machines.md)*
