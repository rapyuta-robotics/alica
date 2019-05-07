@0xe0aa486c26f997e2;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_capnz_msgs");
using import "uuid.capnp".UUID;

struct PlanTreeInfo {
    senderId @0 :UUID;
    stateIds @1 :List(Int64);
    succeededEps @2 :List(Int64);
}