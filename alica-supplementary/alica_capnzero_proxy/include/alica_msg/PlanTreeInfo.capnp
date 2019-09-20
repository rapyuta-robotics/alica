@0xe0aa486c26f997e2;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_msgs");
using IDMsg = import "/capnzero/ID.capnp";

struct PlanTreeInfo {
    senderId @0 :IDMsg.ID;
    stateIds @1 :List(Int64);
    succeededEps @2 :List(Int64);
}