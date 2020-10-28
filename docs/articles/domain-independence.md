# Domain-Independence

The domain-independence of the ALICA Framework allowed to build a substantial [application portfolio](application_portfolio.md) over the time. In this article, we explain how the ALICA Language supports this domain-independence by encapsulating domain-specific elements in distinct core concepts.

The four concepts that include domain-specific content are:

* [conditions](conditions.md)
* [behaviours](behaviours.md)
* [utility functions](utility_functions.md)
* [constraint descriptions](constraints.md)

For these four language elements, ALICA only defines the operational semantics that describes how these elements are handled during the execution of an ALICA program. At the same time, it allows domain experts to fill these elements with domain-specific code which ALICA considers as executable atomic black-box.

Simply put, while the domain expert defines the thresholds for reaching a goal position in a condition, for example, ALICA only knows that this condition can be true or false.

Additionally to these four concepts, the notion of a particular [task](finite-state_machines.md) and [role](roles.md) is also domain-specific, although no domain-specific code needs to be provided in that case. Instead, the ALICA Framework support the modelling of tasks and roles directly.

You want to know how all these element work together? Then we recommend to read the article about [behaviours](behaviours.md) as they are the most fundamental building block of the ALICA Language.

**NAV** *top: [Domain-Independence](domain-independence.md)* *next: [Behaviours](behaviours.md)*