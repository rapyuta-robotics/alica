# Documentation

The documentation of the ALICA Framework is organised in small articles that are readable in 5-10 minutes. Therefore, you can use it as a lookup dictionary. However, we also tried to arrange the articles in an order that allows you to read the documentation as a whole, from start to end. 

The ALICA Framework is further divided into three parts, the language, the engine, and the plan designer. We recommend to understand the language first, since the documentation of the engine and plan designer requires you to understand the semantics of the core concepts of the language.

## The ALICA Language

### Propositional ALICA

#### Core Concepts

[Domain-Independence](./articles/domain-independence.md), [Behaviours](./articles/behaviours.md), [Finite-State Machines](./articles/finite-state_machines.md), [Conditions](./articles/conditions.md), [Entrypoints](./articles/entrypoints.md), [Tasks](./articles/tasks.md), [Plans](./articles/plans.md), [Plantypes](./articles/plantypes.md), [Utility Function](./articles/utility_functions.md)s, Roles

#### Algorithms

Task Allocation, Conflict Resolution, Role Assignment, Transition Synchronisation, Rulebook

### General ALICA

#### Core Concepts

Configurations, Variables, Constraints, Solvers

#### Algorithms

Constraint Queries, Constraint Composition, Constraint Management, Variable Synchronisation

## The ALICA Engine

To be done: 

- Implementation specific things, like AlicaContext, interfaces for clock, solvers, configs, communication, code structure of generated code, etc.
- Maybe implementations in different languages in future (Java and Python are currently work-in-progress)

## The ALICA Plan Designer

The documentation of the ALICA Plan Designer is located in the [repository of the Plan Designer](https://github.com/rapyuta-robotics/alica-plan-designer-fx).