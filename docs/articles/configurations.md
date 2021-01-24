# Configurations

Configurations are simple sets of key-value pairs. They are used to configure behaviours, plans, and plantypes in the context a certain state. A simple Drive behaviour, for example, can be designed to drive to certain point that it expects to be given via a configuration. That way, it is possible to code the Drive behaviour once, but use it to drive to different points. Without configurations, one would need an extra behaviour for each point, which would not scale very well.

The same can be done for plans or plantypes. A whole Navigation plan can , like a simple Drive behaviour, be configured with its destination. 

The keys and values of configurations are simple strings and are expected to be interpreted correctly by the behaviours, plans, and plantypes themselves.

**Please note:** Configuration are reusable concepts. One configuration, that for example describes the destination (x=1, y=3), can be reused to configure multiple different plans, behaviours, and plantypes.

**NAV** *top: [Overview](../README.md)* *next: [Variables](variables.md)*

