@0xe1e7022ab41b6957

struct AllocationAuthorityInfo {
    sender_id @0 :Uint32;
    plan_id @1 :Int64;
    parent_state @2 :Int64;
    plan_type @3 :Int64;
    authority @4 :Uint32;
    entrypoints @5 List(EntryppointRobots);
}

struct EntrypointRobots {
    entrypoint @0 :Int64;
    robots @1 :List(Uint32);
}