# Documentation

The documentation of the ALICA Framework is organised in small articles that are readable in 5-10 minutes. Therefore, you can use it as a lookup dictionary. However, we also tried to arrange the articles in an order that allows you to read the documentation as a whole, from start to end. 

The ALICA Framework is further divided into three parts, the language, the engine, and the plan designer. We recommend to understand the language first, since the documentation of the engine and plan designer requires you to understand the semantics of the core concepts of the language.

## The ALICA Language

### Propositional ALICA

The propositional part of ALICA includes the core concepts that are relevant to all use cases and application scenarios.

1. [Domain-Independence](./articles/domain-independence.md)
2. [Behaviours](./articles/behaviours.md)
3. [Finite-State Machines](./articles/finite-state_machines.md)
4. Entrypoints
5. Conditions
6. Plans
7. Plantypes
8. Utility Functions
9. Roles

### General ALICA

General ALICA extents Propositional ALICA with concepts that enable more sophisticated use cases by utilising configurations, variables, and problem solvers.

1. Configurations
2. Variables
3. Constraints
4. Solvers

## The ALICA Engine

To be done: 

- Implementation specific things, like AlicaContext, interfaces for clock, solvers, configs, communication, code structure of generated code, etc.
- Maybe implementations in different languages in future (Java and Python are currently work-in-progress)

## The ALICA Plan Designer

The documentation of the ALICA Plan Designer is located in the [repository of the Plan Designer](https://github.com/rapyuta-robotics/alica-plan-designer-fx).