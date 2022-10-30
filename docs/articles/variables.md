# Variables

Variables belong to plans, plantypes, or behaviours that provide a context to the variables. There are two kind of variables available in ALICA: Agent variables that are bound to a quantified group of agents, e. g., there is a variable X for each agent A in state S, and free variables that are not bound by any quantification. The only way to assign values to the variables is through a [problem solver](./solvers.md).

## Quantified and Free Variables

A quantifier is a core concept of the ALICA language that is owned by conditions. A quantifier creates quantified variables. Therefore, it defines three things: A quantifier type, a scope, and a set of variables.

**Type:** In theory the ALICA language supports different types of quantifiers. In practise only the universal quantifier for agents is supported.

**Scope:** The scope of a quantifier relates to the plan in which the condition that owns the quantifier belongs to. Therefore, the scope of a quantifier can either be the whole plan, a finite-state machine (identified by its task), or a state in that plan.

**Variable Set:** The variables are, at design time, only a single string property of quantifiers. This string is a comma separated list of variable names.

So the semantics of a quantifier can be read as "For all agents (universal agent quantifier) in that scope (plan, FSM, or state), there exist these variables (list of variable names)." At runtime of the ALICA engine, all quantifiers are parsed and the corresponding variables are created, based on the composition of the team of agents. This creation is fundamentally different from free variables, which are directly attached to plans, planytpes, or behaviours. Free variables always exist independent from the composition of the team. Quantified variables exist independent from the quantifiers and their contexts in which they are created. If one quantifier says that there are variables "pos_x" and "pos_y" for each agent, then every other quantifier which is declaring the same two variables in a completely different context, is talking about the exact same two variables as the first quantifier did. Thus quantified variables are considered to be global.

## Variable Bindings

The variables of plans and plantypes in ALICA programs can be bound to variables of other plans, plantypes, and behaviours via a variable binding. The purpose of variable bindings in the ALICA language is to combine different constraints for the same variable in the plan hierarchy of an ALICA program.
There are two objects that can include variable bindings in the ALICA language. States, on the one hand, use bindings to bind the variables of the plan they are part of to variables of plans, plantypes, or behaviours inside themselves. Plantypes, on the other hand, bind their own variables to variables of plans inside themselves. The variable bindings are transitive, i. e. a variable of the top-level plan can be bound to variables of plans several levels below in the plan tree structure.

A variable binding includes three entities: a variable, a sub-variable, and a sub plan.

**Variable:** The variable in a variable binding, is the variable that should be bound to the sub-variable. The variable always belongs to the plan or the plantype which (indirectly) owns the variable binding and therefore, the variable does not need any additional context.

**Sub-Variable:** The sub-variable is a variable which belongs to the next deeper level in the plan hierarchy. For example, it belongs to a child plan in a state.

**Sub-Plan:** The sub-plan is the owner of the sub-variable, which is not always a plan. Since states can include multiple plans, behaviours, and plantypes and all can own variables, a sub-plan can be one of these three types.

Please note, that variable bindings can only bind free variables. Quantified variables cannot be bound at all, since they are considered to be globally defined for each agent and not bound to the context of the plan, which owns the quantifier.

**NAV** _prev:_ [Configurations](configurations.md) _top: [Overview](../README.md)_ _next: [Constraints](constraints.md)_
