#pragma once

#include <engine/containers/AlicaEngineInfo.h>
#include <engine/containers/AllocationAuthorityInfo.h>
#include <engine/containers/PlanTreeInfo.h>
#include <engine/containers/SyncReady.h>
#include <engine/containers/SyncTalk.h>
#include <engine/containers/SolverResult.h>

#include <capnzero/CapnZero.h>
#include <engine/containers/RoleSwitch.h>

namespace essentials {
    class IDManager;
}

namespace alica_capnzero_proxy {
    class ContainerUtils {
    public:
        static alica::AllocationAuthorityInfo toAllocationAuthorityInfo(::capnp::FlatArrayMessageReader& msg, essentials::IDManager* idManager);
        static alica::AlicaEngineInfo toAlicaEngineInfo(::capnp::FlatArrayMessageReader& msg, essentials::IDManager* idManager);
        static alica::PlanTreeInfo toPlanTreeInfo(::capnp::FlatArrayMessageReader& msg, essentials::IDManager* idManager);
        static alica::SyncReady toSyncReady(capnp::FlatArrayMessageReader &msg, essentials::IDManager *idManager);
        static alica::SyncTalk toSyncTalk(capnp::FlatArrayMessageReader &msg, essentials::IDManager *idManager);
        static alica::SolverResult toSolverResult(capnp::FlatArrayMessageReader &msg, essentials::IDManager *idManager);

        static void toMsg(alica::AllocationAuthorityInfo aai, ::capnp::MallocMessageBuilder& msgBuilder);
        static void toMsg(alica::AlicaEngineInfo aei, ::capnp::MallocMessageBuilder& msgBuilder);
        static void toMsg(alica::PlanTreeInfo pti, ::capnp::MallocMessageBuilder& msgBuilder);
        static void toMsg(alica::RoleSwitch rs, ::capnp::MallocMessageBuilder& msgBuilder);
        static void toMsg(alica::SyncReady sr, ::capnp::MallocMessageBuilder& msgBuilder);
        static void toMsg(alica::SyncTalk st, ::capnp::MallocMessageBuilder& msgBuilder);
        static void toMsg(alica::SolverResult sr, ::capnp::MallocMessageBuilder& msgBuilder);

    private:
        ContainerUtils() = delete;
    };
}