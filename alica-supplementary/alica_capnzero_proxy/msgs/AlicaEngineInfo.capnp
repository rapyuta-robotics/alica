@0xf1cf2b46ae0692ca;

struct AlicaEngineInfo {
    senderId @0 :List(UInt8);
    masterPlan @1 :Text;
    currentPlan @2 :Text;
    currentState @3 :Text;
    currentRole @4 :Text;
    currentTask @5 :Text;
    agentIdsWithMe @6 :List(List(UInt8));
}