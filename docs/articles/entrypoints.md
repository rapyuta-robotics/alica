# Entrypoints

The [finite-state machines](finite-state_machines.md) (FSM) in the ALICA language always have a single initial state. Every agent that starts the execution of a FSM starts with this initial state. An entrypoint identifies the initial state of a FSM and declares how many agents are allowed to execute a FSM at the same time. 

The *minimum cardinality* of an entrypoint defines the minimum number of agents that are necessary to execute a FSM. The minimum cardinality of a FSM is always at least 1. If an agent does not believe that enough agents will start the execution of a FSM with it, the agent does not start the execution at all. The ALICA Engine takes care of synchronising all agents about their intents of executing FSMs. 

The *maximum cardinality* of an entrypoint defines the maximum number of agents that are allowed to execute a FSM. If a FSM is full, no other agent can start executing it. Again the ALICA Engine takes care of synchronising the number of agents in a FSM.

Another core concept that is attached to an entrypoint is the concept of [tasks](tasks.md) in the ALICA language. Tasks are explained in their own article.

