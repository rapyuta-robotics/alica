@0x802ab5ece8a1ce5a;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_capnp_msgs");
using import "uuid.capnp".UUID;

struct RoleSwitch {
    senderId @0 :UUID;
    roleId @1 :Int64;
}