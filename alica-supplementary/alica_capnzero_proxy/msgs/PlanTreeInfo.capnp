@0xe0aa486c26f997e2;

struct PlanTreeInfo {
    senderId @0 :List(UInt8);
    stateIds @1 :List(Int64);
    succeededEps @2 :List(Int64);
}