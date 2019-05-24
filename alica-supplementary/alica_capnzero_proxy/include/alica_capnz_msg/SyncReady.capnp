@0xcc2514efa5bd219a;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_capnz_msgs");
using IDMsg = import "/capnzero/ID.capnp";

struct SyncReady {
    senderId @0 :IDMsg.ID;
    synchronisationId @1 :Int64;
}