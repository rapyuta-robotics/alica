#pragma once

#include "supplementary/AgentID.h"
#include <tuple>

namespace alica
{
typedef std::tuple<const supplementary::AgentID*, int64_t> stdSyncReady;
struct SyncReady
{
    SyncReady()
        : senderID(nullptr)
        , syncTransitionID(0)
    {
    }
    const supplementary::AgentID* senderID;
    int64_t syncTransitionID;

    SyncReady(const stdSyncReady& s)
        : senderID(std::get<0>(s))
        , syncTransitionID(std::get<1>(s))
    {
    }

    stdSyncReady toStandard() { return std::make_tuple(senderID, syncTransitionID); }
};

} /* namespace alica */
