# Configurations

Configurations are simple sets of key-value pairs. They are used to configure behaviours, plans, and plantypes in the context a certain state. A simple *Drive* behaviour, for example, can be designed to drive to certain point that it expects to be given via a configuration. That way, it is possible to code the *Drive* behaviour once, but use it to drive to different points. Without configurations, one would need an extra behaviour for each point, which would not scale very well.

The same can be done for plans or plantypes. A whole Navigation plan, for example, can be configured with its destination, maximum velocity, and forbidden areas of the map. 

The keys and values of configurations are simple strings and are expected to be interpreted correctly by the behaviours, plans, and plantypes themselves.

Since it is possible to configure behaviours, plans, and plantypes in the context of states, it is also possible to insert the same behaviour, plan, or plantype twice into the same state, if it is configured differently. Consider the example of a humanoid robot. Equipped with two arms is is very likely, that it has to move its arms concurrently in the same state of a plan. In such a case, it is possible to reuse the same *Move Arm* plan twice, just configured with the right or left arm, respectively. Compared to the [Plan Trees](./plan_trees.md) article, this adds the configuration parameter to the edges in the plan tree:

* **Behaviours:** Plan -- (Task, State, Configuration) --> Behaviour
* **Plans:** Plan -- (Task, State, Configuration) --> Plan
* **Plantypes:** Plan -- (Task, State, Plantype, Configuration) --> Plan

**Please note:** Configuration are reusable concepts. One configuration, that for example describes the destination (x=1, y=3), can be reused to configure multiple different plans, behaviours, and plantypes.

**NAV** *top: [Overview](../README.md)* *next: [Variables](variables.md)*

