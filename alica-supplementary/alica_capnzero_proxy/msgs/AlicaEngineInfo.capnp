@0xf1cf2b46ae0692ca;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_capnz_msgs");
using import "uuid.capnp".UUID;

struct AlicaEngineInfo {
    senderId @0 :UUID;
    masterPlan @1 :Text;
    currentPlan @2 :Text;
    currentState @3 :Text;
    currentRole @4 :Text;
    currentTask @5 :Text;
    agentIdsWithMe @6 :List(UUID);
}