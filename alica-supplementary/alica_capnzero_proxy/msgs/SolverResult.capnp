@0xfcdd9ec652f24502;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_capnp_msgs");
using import "uuid.capnp".UUID;

struct SolverResult {
    senderId @0 :UUID;
    vars @1 :List(SolverVar);
}

struct SolverVar {
    id @0 :Int64;
    value @1 :List(UInt8);
}