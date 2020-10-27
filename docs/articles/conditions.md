# Conditions

Conditions are just like behaviours, one of the four core concepts in the ALICA Framework, that encapsulate domain-specific code. Hence, the domain-independent ALICA Framework handles conditions as black-boxes. From the Frameworks' perspective, conditions can be true or false and the conditions must provide a method to determine their current truth value.

Further, the ALICA language distinguishes between three different kinds of conditions: pre-, runtime, and postconditions. A precondition must hold before something is done. The runtime condition must hold before and while something is done. And finally, the postcondition is only expected to hold after something has been done. 

Please note the different semantics between postconditions and the other two conditions. While pre- and runtime conditions control the progress of an ALICA program, the postconditions have no influence on that at all. Their simple purpose is to allow the designer of an ALICA program to express what outcome is expected from a certain element. This, for example, allows the application of classic planning algorithms to the ALICA language.

TODO:

- Table were which condition is used 
- Explain the specific semantics for the different places