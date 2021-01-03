# Utility Functions (WIP)
A utility function represents criteria according to which a plan is more or less suitable for a certain situation. $U_p(B)$ is the utility function of plan $p$ parametrised with the believe base $B$. A believe base of an agent can include everything that it believes about its domain and current situation, as well as everything that it believes about the current execution state of the running ALICA program. However, a utility function may only refer to parts of the ALICA program that belong to the plan $p$, in order to fulfil the locality principle which guarantees the scaleability of ALICA programs at runtime. 

Utility functions are evaluated at runtime, in order to choose the best plan from a plantype and choose the best assignment of tasks to agents. Therefore, the believe base $B$ always includes the current or a potential task assignment for the plan $p$.

The form of a utility function has always the following form:
$$
U_p(B) = w_0*pri(B) + w_1*sim(B) + w_2*f_2(B)+...+w_n*f_n(B)
$$
A utility function, as given in Equation (1), is a weighted sum of functions. The weights are constants between 0 and 1 that sum up to 1. 

The functions represent independent criteria according to which the plan is suitable for the current situation or not. A utility function includes the priority function $pri(B)$ and the similarity function $sim(B)$ by default, but their weights can be set to 0 as well.

## Role-Task Priority Function

The priority function $pri(B)$ evaluates the assignment of [tasks](tasks.md) to agents for plan $p$ according to the preferences of the agents' roles. An agent that has a *transporter* role, for example, is well suited for the *transport* task. How much which role prioritises which task is given is defined by each [role](roles.md) itself

## Similarity Function

The similarity function $sim(B)$ evaluates how similar two assignments of tasks to agents for plan $p$ are. The purpose of the similarity function is to converge the assignment of tasks to agents at runtime for the whole team of agents.

**NAV** *prev: [Plantypes](plantypes.md)*  *top: [Overview](README.md)* *next: [Roles](roles.md)*