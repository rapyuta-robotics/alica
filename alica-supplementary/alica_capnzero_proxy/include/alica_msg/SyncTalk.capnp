@0x88f4b2eb0205ebe8;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_msgs");
using IDMsg = import "/capnzero/ID.capnp";

struct SyncTalk {
    senderId @0 :IDMsg.ID;
    syncData @1 :List(SyncData);
}

struct SyncData {
    robotId @0 :IDMsg.ID;
    transitionId @1 :Int64;
    transitionHolds @2 :Bool;
    ack @3 :Bool;

}