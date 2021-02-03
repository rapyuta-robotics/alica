# Constraints

The core concept of a constraint in the ALICA language is very general, but for an easier understanding, think of it as in the domain of [Constraint Satisfaction Problems (CSP)](https://en.wikipedia.org/wiki/Constraint_satisfaction_problem). In CSPs a constraint restricts the range of a variable, e.g., "X must be large then 5 and smaller then 8" is a simple linear constraint. 

However, the ALICA framework allows you to use arbitrary formalisms to constrain [variables](./variables.md). At the moment, there is a set of formalisms supported a [corresponding set of solvers](./solvers.md). 

Independent from the utilised formalism, the a constraint in the ALICA language limits the range of values assignable to variables, but constraints
only exist in the context of preconditions and runtime conditions (not postconditions) of ALICA programs. Therefore, each precondition and runtime condition may have an optional set of associated constraints, and each precondition and runtime condition defines which variables may be referenced by its constraints. Furthermore, the conditions guard the constraints in a way that solutions to constraints are asserted (not proven), if and
only if its guarding condition holds. The conditions are denoted as weak guards for the constraints, while a strong guard would require to prove the existence of a solution for the constraints. In ALICA the weak guard interpretation is preferred because proving the existence of a solution to a constraint satisfaction problem (CSP) can be intractable, depending on the expressiveness of the utilised constraint formalism.

**NAV** *prev: [Variables](variables.md) top: [Overview](../README.md) next: [Solvers](solvers.md)*