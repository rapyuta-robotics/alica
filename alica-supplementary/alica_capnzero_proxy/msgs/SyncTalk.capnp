@0x88f4b2eb0205ebe8;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_capnp_msgs");
using import "uuid.capnp".UUID;

struct SyncTalk {
    senderId @0 :UUID;
    syncData @1 :List(SyncData);
}

struct SyncData {
    robotId @0 :UUID;
    transitionId @1 :Int64;
    transitionHolds @2 :Bool;
    ack @3 :Bool;
}