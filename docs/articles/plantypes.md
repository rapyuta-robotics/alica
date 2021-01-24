# Plantypes
A plantype represents a certain type of plans, e.g., all plans that will clean up your household. In the [plans article](./plans.md) one example is given for such a clean up plan. However, there are many other ways of cleaning up a household. A plantype is thought to represent such a set of different options to achieve a certain goal. 

In fact a plantype is just a set of plans. At runtime, the ALICA Engine executes the plan that suits the current situation the best, by evaluating the utility functions of all plans in the plantype. Please note, that one plan can be included in many plantypes.

Further, as a convenience function, it is possible to deactivate plans for individual plantypes without removing them.  This is very handy, when you swiftly want to change the options your robots can choose from during development.

**NAV** *prev: [Plans](plans.md)*  *top: [Overview](../README.md)* *next: [Utility Functions](utility_functions.md)*