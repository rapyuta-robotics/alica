# Constraints

The core concept of a constraint in the ALICA language is very general, but for an easier understanding, think of it as in the domain of [Constraint Satisfaction Problems (CSP)](https://en.wikipedia.org/wiki/Constraint_satisfaction_problem). In CSPs a constraint restricts the range of a variable, e.g., "X must be large then 5 and smaller then 8" is a simple linear constraint. 

However, the ALICA framework allows you to use arbitrary formalisms to constrain [variables](./variables.md). At the moment, there is a set of formalisms supported a [corresponding set of solvers](./solvers.md). 

Independent from the utilised formalism, the a constraint in the ALICA language limits the range of values assignable to variables, but constraints
only exist in the context of preconditions and runtime conditions (not postconditions) of ALICA programs. Therefore, each precondition and runtime condition may have an optional set of associated constraints, and each precondition and runtime condition defines which variables may be referenced by its constraints. Furthermore, the conditions guard the constraints in a way that solutions to constraints are asserted (not proven), if and
only if its guarding condition holds. The conditions are denoted as weak guards for the constraints, while a strong guard would require to prove the existence of a solution for the constraints. In ALICA the weak guard interpretation is preferred because proving the existence of a solution to a constraint satisfaction problem (CSP) can be intractable, depending on the expressiveness of the utilised constraint formalism.

## Solvers

Solvers are the only way to assign values to [variables](./variables.md) in the ALICA framework. The ALICA engine defines a general interface, which it expects to be implemented by any solver that wants to be compatible to the ALICA framework.

| Solver                                                       | Description                                                  | Maintenance State            |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ---------------------------- |
| [Simple Solver](https://github.com/rapyuta-robotics/alica/tree/rr-devel/alica_simple_solver) | Simply assigns values to variables, without applying any kind of algorithm. Also utilises a [black board](https://github.com/rapyuta-robotics/alica/tree/rr-devel/alica_engine/include/engine/blackboard) metaphor for sharing values in the team. | Actively maintained.         |
| [CGSolver](https://github.com/rapyuta-robotics/alica-supplementary/tree/rr-devel/constraintsolver) | Solves continuous nonlinear constraint satisfaction problems for variables in the domain of real numbers. It is based on a C++ port of the [AutoDiff](https://github.com/alexshtf/autodiff) library. It also includes a satisfiability modulo theory extention, described in a [research paper 2013](https://ieeexplore.ieee.org/document/6697046). | Actively maintained.         |
| [ASPSolver](https://github.com/dasys-lab/aspsuite/tree/srg_dev) | The answer set programming (ASP) solver [Clingo](https://github.com/potassco/clingo), made compatible to the ALICA framework. Variable synchronisation among team members not supported. | 6 Month old, not maintained. |
| [CACE](https://github.com/dasys-lab/cace)                    | A consensus framework build with a distributed tuple space metaphor in mind. Not an actual solver, but is designed to assign values to variables based on the consensus achieved by the team of agents. | 6 Years old, not maintained. |

*Table 1: All known ALICA-compatible solvers.*

Table 1 shows the solvers that are known to be compatible to the ALICA solver interface. 

**NAV** *prev: [Variables](variables.md) top: [Overview](../README.md)*