# Conflict Resolution

The conflict resolution of the ALICA runtime engine was published in 2011 [1]. Its job is to resolve ongoing or reoccurring conflicts between agents with regard to the assignment of tasks to agents. However, before those conflicts can be resolved, it is necessary to detect them. This is the job of the Cycle Manager. Each *Running Plan*, which is the runtime representation of a plan, plantype, or behaviour in progress, includes its own Cycle Manager. Obviously, there can only be allocation conflicts for plans and plantypes.

## Cycle Detection

The cycle manager has a ring buffer of allocation changes paired with the reason for this change of the allocation. The reason can either be *none*, an allocation *message* announced by a another agent, or a local allocation change that only happens when the *utility* of the current allocation is worse than a different allocation.  This history of allocation changes and reasons is checked for cycle.

A cycle in this context has the following properties:

* It starts with an allocation change that has an improved *utility* as reason,
* followed by an arbitrary number of allocation changes induced by received *messages*,
* and if all allocation changes of that cycle are accumulated, actually no change happened.

Let's put this simple, if the local agent changes the allocation due to an improved utility and afterwards an arbitrary number of messages, received from its teammates, revert this change completely, we have a conflict between the local agent and its teammates. If these kind of cycles are detected more than a configurable amount of times, the Cycle Manager signals that the Authority Manager should kick in and override the allocations of the involved teammates.

## Authority Manager 

The Authority Manager asks the Cycle Manager, if it should override the task allocation because of reoccurring conflict cycles. If that is the case, the question is which agent should override the allocation and which agents should obey. The ALICA runtime engine follows a simple [Bully algorithm](https://en.wikipedia.org/wiki/Bully_algorithm) in which the *leader* or winner of the election, is the agent with the lowest ID. 

Once the involved agent with the lowest ID is identified, it sends dedicated authority messages to the other involved agents stating the allocation that they should adapt together with some auxiliary information like its own ID, the state in which the plan instance that had the conflict is running.

The aforementioned algorithms can be fine tuned with the following parameters.

| Name                         | Typical Value                | Description                                                  |
| ---------------------------- | :--------------------------- | ------------------------------------------------------------ |
| Enabled                      | true                         | Enables or disables the Conflict Resolution                  |
| CycleCount                   | 5                            | The number of cycles that have to be detected within the allocation history, before the Authority Manager starts the conflict resolution. |
| MinimalAuthorityTimeInterval | 800 [ms]                     | The minimal time interval for which the allocation is overridden by the Authority Manager, if enough cycles are detected. |
| MaximalAuthorityTimeInterval | 5000 [ms]                    | The maximal time interval for which the allocation is overridden by the Authority Manager, if enough cycles are detected. |
| IntervalIncreaseFactor       | 1.5                          | Every time enough cycles are detected, the time for which the Authority Manager will be active is increase by this factor, but not above the MaximalAuthorityTimeInterval. |
| IntervalDecreaseFactor       | 0.999 [about 1 min cooldown] | Every time not enough cycles are detected, the time for which the Authority Manager will be active is decreased by this factor, but not below the MinimalAuthorityTimeInterval. |
| MessageTimeInterval          | 60 [ms]                      | How often the allocation override message should be sent by the agent that has the authority. |
| MessageWaitTimeInterval      | 200                          | NOT USED!?                                                   |
| HistorySize                  | 45                           | Size of the ring buffer that holds the allocation change history. |

*Table 1:* Configuration Parameters for the Conflict Resolution

[1] Skubch, Hendrik, Daniel Saur, and Kurt Geihs. "[Resolving conflicts in highly reactive teams](https://drops.dagstuhl.de/opus/volltexte/2011/2968/)." 17th GI/ITG Conference on Communication in Distributed Systems (KiVS 2011). Schloss Dagstuhl-Leibniz-Zentrum fuer Informatik, 2011.

**NAV**  *prev: [Task Allocation](task_allocation.md)* *top: [Overview](../README.md)* *next: [Role Assignment](role_assignment.md)*

