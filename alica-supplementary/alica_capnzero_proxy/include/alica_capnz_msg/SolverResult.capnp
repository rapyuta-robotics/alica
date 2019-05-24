@0xfcdd9ec652f24502;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_capnz_msgs");
using IDMsg = import "/capnzero/ID.capnp";

struct SolverResult {
    senderId @0 :IDMsg.ID;
    vars @1 :List(SolverVar);
}

struct SolverVar {
    id @0 :Int64;
    value @1 :List(UInt8);
}