#pragma once

#include "engine/IRobotID.h"
#include <tuple>

namespace alica
{
typedef std::tuple<const alica::IRobotID *, long> stdSyncReady;
struct SyncReady
{
    SyncReady() : senderID(nullptr), syncTransitionID(0)
    {
    }
    const alica::IRobotID *senderID;
    long syncTransitionID;

    SyncReady(stdSyncReady &s)
    {
        this->senderID = std::get<0>(s);
        this->syncTransitionID = std::get<1>(s);
    }

    stdSyncReady toStandard()
    {
        return std::move(std::make_tuple(senderID, syncTransitionID));
    }
};

} /* namespace alica */
