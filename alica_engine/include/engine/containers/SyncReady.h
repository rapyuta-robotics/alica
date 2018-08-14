#pragma once

#include <engine/AgentIDConstPtr.h>

#include <tuple>

namespace alica
{
typedef std::tuple<AgentIDConstPtr, int64_t> stdSyncReady;
struct SyncReady
{
    SyncReady()
            : senderID(nullptr)
            , syncTransitionID(0)
    {
    }
    AgentIDConstPtr senderID;
    int64_t syncTransitionID;

    SyncReady(const stdSyncReady& s)
            : senderID(std::get<0>(s))
            , syncTransitionID(std::get<1>(s))
    {
    }

    stdSyncReady toStandard() { return std::make_tuple(senderID, syncTransitionID); }
};

} /* namespace alica */
