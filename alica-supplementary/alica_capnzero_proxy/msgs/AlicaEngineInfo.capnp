@0xf1cf2b46ae0692ca

struct AlicaEngineInfo {
    sender_id @0 :Uint32;
    master_plan @1 :Text;
    current_plan @2 :Text;
    current_state @3 :Text;
    current_role @4 :Text;
    current_task @5 :Text;
    agent_ids_with_me @6 :List(Uint32);
}