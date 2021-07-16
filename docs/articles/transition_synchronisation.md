# Transition Synchronisation

*The following explanation is partially based on the PhD Thesis of Hendrik Skubch [2]*

The purpose of synchronised transitions is already explained in the [corresponding article](synchronisations.md). Nevertheless, the actual synchronisation is the job of the Sync module.

The protocol, implemented by the Sync module, is a three-way handshake. Each participating agent announces its readiness to synchronise if

* it inhabits a state that has an outgoing transition, which is synchronised with other transitions via a Synchronisation 
* it believes the precondition of that outgoing transition to hold. 

Each participating agent acknowledges the announcement, and broadcasts a readiness signal once it received an acknowledgement of all announcements from every participating agents. In that moment it starts to support the mutual belief that the team is going to follow the synchronised transitions and is able to follow its own synchronised transition. Should an agent receive a readiness signal while believing its respective precondition holds, it immediately supports the mutual belief. Should contradictory information arise during the establishment of the belief, the corresponding agent informs the group to retract its commitment.

Establishing mutual belief in a synchronised manner through communication comes down to solving the coordinated attack problem [1], which is known to have no finite solution, given asynchronous, unreliable communication. Therefore, any protocol for synchronised transitions can only be approximately correct. We deem a three-way handshake acceptable under most conditions, however, it is easy to extend this basic protocol to an n-way handshake.

[1] Piotr J. Gmytrasiewicz and Edmund H. Durfee. [Decision-theoretic recursive modeling and the coordinated attack problem](http://dl.acm.org/citation.cfm?id=139492.139503). In Proceedings of the first international conference on Artificial intelligence planning systems, pages 88–95, San Francisco, CA, USA, 1992. Morgan Kaufmann Publishers Inc. ISBN 1-55860-250-X.

[2] Skubch, Hendrik. *[Modelling and controlling of behaviour for autonomous mobile robots](https://www.springer.com/de/book/9783658008109)*. Springer Science & Business Media, 2012.

**NAV**  *prev: [Role Assignment](rule_book.md)* *top: [Overview](../README.md)* *next: [Rulebook](rulebook.md)*

