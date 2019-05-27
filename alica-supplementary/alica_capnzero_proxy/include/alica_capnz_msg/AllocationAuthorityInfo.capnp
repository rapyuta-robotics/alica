@0xe1e7022ab41b6957;
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("alica_capnz_msgs");

using IDMsg = import "/capnzero/ID.capnp";

struct AllocationAuthorityInfo {
    senderId @0 :IDMsg.ID;
    planId @1 :Int64;
    parentState @2 :Int64;
    planType @3 :Int64;
    authority @4 :IDMsg.ID;
    entrypointRobots @5 :List(EntrypointRobots);

}

struct EntrypointRobots {
    entrypoint @0 :Int64;
    robots @1 :List(IDMsg.ID);
}