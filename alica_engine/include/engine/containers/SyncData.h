#pragma once
#include <tuple>

namespace supplementary {
	class IAgentID;
}

namespace alica
{
typedef std::tuple<const supplementary::IAgentID *, long, bool, bool> stdSyncData;

struct SyncData
{
    SyncData()
        : robotID(nullptr)
        , ack(false)
        , conditionHolds(false)
    	, transitionID(0)
    {
    }

    const supplementary::IAgentID *robotID;
    long transitionID;
    bool conditionHolds;
    bool ack;

    SyncData(stdSyncData &s)
    {
        this->robotID = std::get<0>(s);
        this->transitionID = std::get<1>(s);
        this->conditionHolds = std::get<2>(s);
        this->ack = std::get<3>(s);
    }

    stdSyncData toStandard()
    {
        return std::move(std::make_tuple(robotID, transitionID, conditionHolds, ack));
    }

    void toString()
    {
    	std::cout << "SyncData--> ";
    	std::cout << " RobotId: " << this->robotID;
    	std::cout << " TransitionID: " << this->transitionID;
    	std::cout << " ConditionHolds: " << this->conditionHolds;
    	std::cout << " Acknowledge: " << this->ack << std::endl;
    }
};

} /* namespace alica */
