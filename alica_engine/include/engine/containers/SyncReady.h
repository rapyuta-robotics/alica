#pragma once

#include "supplementary/AgentID.h"
#include <tuple>

namespace alica {
typedef std::tuple<const supplementary::AgentID*, long> stdSyncReady;
struct SyncReady {
    SyncReady()
            : senderID(nullptr)
            , syncTransitionID(0) {}
    const supplementary::AgentID* senderID;
    long syncTransitionID;

    SyncReady(stdSyncReady& s) {
        this->senderID = std::get<0>(s);
        this->syncTransitionID = std::get<1>(s);
    }

    stdSyncReady toStandard() {
        return std::move(std::make_tuple(senderID, syncTransitionID));
    }
};

} /* namespace alica */
