@0x88f4b2eb0205ebe8;

struct SyncTalk {
    senderId @0 :List(UInt8);
    syncData @1 :List(SyncData);
}

struct SyncData {
    robotId @0 :List(UInt8);
    transitionId @1 :Int64;
    transitionHolds @2 :Bool;
    ack @3 :Bool;
}