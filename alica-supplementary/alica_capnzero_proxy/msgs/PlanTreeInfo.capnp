@0xe0aa486c26f997e2

struct PlanTreeInfo {
    sender_id @0 :Uint32;
    state_ids @1 :List(Int64);
    succeeded_eps @2 :List(Int64);
}