@0xcc2514efa5bd219a;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_capnp_msgs");
using import "uuid.capnp".UUID;

struct SyncReady {
    senderId @0 :UUID;
    synchronisationId @1 :Int64;
}