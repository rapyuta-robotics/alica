# Documentation

The documentation of the ALICA Framework is organised in small articles that are readable in 5-10 minutes. Therefore, you can use it as a lookup dictionary. However, we also tried to arrange the articles in an order that allows you to read the documentation as a whole, from start to end. 

The ALICA Framework is further divided into three parts, the language, the engine, and the plan designer. We recommend to understand the language first, since the documentation of the engine and plan designer requires you to understand the semantics of the core concepts of the language.

## The ALICA Language

### Propositional ALICA

The propositional part of the ALICA language is expressive enough to describe complex strategies for teams of autonomous robots. Its core concepts are explained in the following articles, followed by the algorithms that make up the runtime behaviour of the ALICA engine with regard to the propositional core concepts.

#### Core Concepts

[Domain-Independence](./articles/domain-independence.md), [Behaviours](./articles/behaviours.md), [Finite-State Machines](./articles/finite-state_machines.md), [Conditions](./articles/conditions.md), [Entrypoints](./articles/entrypoints.md), [Tasks](./articles/tasks.md), [Plans](./articles/plans.md), [Synchronisations](./articles/synchronisations.md), [Plantypes](./articles/plantypes.md), [Utility Functions](./articles/utility_functions.md), [Roles](./articles/roles.md), [Plan Trees](./articles/plan_trees.md)

#### Algorithms

[Task Allocation](./articles/task_allocation.md), [Conflict Resolution](./articles/conflict_resolution.md), [Role Assignment](./articles/role_assignment.md), [Transition Synchronisation](./articles/transition_synchronisation.md), [Rulebook](./articles/rulebook.md)

### General ALICA

General ALICA adds several core concepts to the propositional part of the ALICA language in order to increase the generality and reusability of ALICA programs. Before you dive into the core concepts of General ALICA, a short [article motivates](./articles/motivation_general_alica.md) it with a practical example.

#### Core Concepts

[Configurations](./articles/configurations.md), [Variables](./articles/variables.md), [Constraints](./articles/constraints.md)

#### Algorithms

[Constraint Queries](./articles/constraint_query.md), [Constraint Composition](./articles/constraint_composition.md), [Constraint Management](./articles/constraint_management.md), [Variable Synchronisation](./articles/variable_synchronisation.md)

## The ALICA Engine

To be done: 

- Implementation specific things, like AlicaContext, interfaces for clock, solvers, configs, communication, code structure of generated code, etc.
- Maybe implementations in different languages in future (Java and Python are currently work-in-progress)

## The ALICA Plan Designer

The documentation of the ALICA Plan Designer is located in the [repository of the Plan Designer](https://github.com/rapyuta-robotics/alica-plan-designer-fx).