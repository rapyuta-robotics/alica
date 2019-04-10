@0xe1e7022ab41b6957;

struct AllocationAuthorityInfo {
    senderId @0 :List(UInt8);
    planId @1 :Int64;
    parentState @2 :Int64;
    planType @3 :Int64;
    authority @4 :List(UInt8);
    entrypoints @5 :List(EntrypointRobots);

}

struct EntrypointRobots {
    entrypoint @0 :Int64;
    robots @1 :List(List(UInt8));
}