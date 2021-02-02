# Motivating Example for General ALICA

Propositional ALICA with its modelling elements is already expressive enough to describe complex strategies for teams of autonomous robots. Nevertheless, modelling within the constraints of Propositional ALICA introduces limits regarding to the generality and reusability of the modelled ALICA programs. Considering the plan in Figure 1, the question is: "How to specify the object that needs to be lifted?" 

![synchronisation_plan_example](../images/synchronsation_plan_example.svg)

*Figure 1: A Plan with Two Synchronised Transitions*

So far in Propositional ALICA, two options exist that are explained in the following paragraphs. The object could be hard-coded into the plan and behaviours, albeit limiting their reusability. Two behaviours for each object (left and right side to grab) would be necessary and if unknown objects are encountered at runtime the behaviours and plans need to be generated and planned at runtime, respectively. Another option is to encapsulate the encoding of the object in the black box part of the behaviours. An algorithm that determines the kind of object that needs to be lifted, for example, gets sensor data as input and configures the parameters of behaviours accordingly. A disadvantage of this approach would be the separation of the ALICA program structure from the logic that interacts and reasons about the object. The lift behaviours in state Z 1 and Z 3, for example, wouldn’t be able to reference the object that was just lifted. In order to guarantee that the same object is lifted that was just grasped, the black box part of the behaviour would need to establish a connection that already exists in the plan itself, represented through the transition between the Grab and Lift states.

General ALICA extends Propositional ALICA in a way that allows to reference the same objects from different states and implements several further mechanism that improve the expressiveness of ALICA programs by combining knowledge about the ALICA program structure and about the current situation. The modelling elements and modules necessary for this, are explained in the following articles.

**NAV** *top: [Overview](../README.md) next: [Configurations](./configurations.md)*