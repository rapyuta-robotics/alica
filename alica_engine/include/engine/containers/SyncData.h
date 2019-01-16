#pragma once

#include <engine/AgentIDConstPtr.h>

#include <iostream>
#include <tuple>

namespace alica
{
typedef std::tuple<AgentIDConstPtr, int64_t, bool, bool> stdSyncData;

struct SyncData
{
    SyncData()
            : robotID(nullptr)
            , ack(false)
            , conditionHolds(false)
            , transitionID(0)
    {
    }

    SyncData(const stdSyncData& s)
            : robotID(std::get<0>(s))
            , transitionID(std::get<1>(s))
            , conditionHolds(std::get<2>(s))
            , ack(std::get<3>(s))
    {
    }

    stdSyncData toStandard() const { return std::make_tuple(robotID, transitionID, conditionHolds, ack); }

    void toString() const
    {
        std::cout << "SyncData--> ";
        std::cout << " RobotId: " << robotID;
        std::cout << " TransitionID: " << transitionID;
        std::cout << " ConditionHolds: " << conditionHolds;
        std::cout << " Acknowledge: " << ack << std::endl;
    }

    AgentIDConstPtr robotID;
    int64_t transitionID;
    bool conditionHolds;
    bool ack;
};

} /* namespace alica */
