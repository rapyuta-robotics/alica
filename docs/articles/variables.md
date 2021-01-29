# (WIP) Variables

Variables belong to plans, plantypes, or behaviours that provide a context to the variables. There are two kind of variables available in ALICA: Agent variables that are bound to a quantified group of agents, e. g., there is a variable X for each agent A in state S, and free variables that are not bound by any quantification. The only way to assign values to the variables is through a [problem solver](./solvers.md). 

## Variable Bindings

The variables of plans and plantypes in ALICA programs can be bound to variables of other plans, plantypes, and behaviours via a variable binding. The purpose of variable bindings in ALICA is to combine different constraints for the same variable in the plan hierarchy of an ALICA program. 
There are two objects that can include variable bindings in the ALICA language. States, on the one hand, use bindings to bind the variables of the plan they are part of to variables of plans, plantypes, or behaviours inside themselves. Plantypes, on the other hand, bind their own variables to variables of plans inside themselves. The variable bindings are transitive, i. d. that a variable of the top-level plan can be bound to variables of plans several levels below in the plan tree structure.

**NAV** *prev:* [Configurations](configurations.md) *top: [Overview](../README.md)* *next: [Constraints](constraints.md)*

