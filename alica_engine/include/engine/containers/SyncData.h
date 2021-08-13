#pragma once

#include <essentials/IdentifierConstPtr.h>

#include <iostream>
#include <tuple>

namespace alica
{
typedef std::tuple<essentials::IdentifierConstPtr, int64_t, bool, bool> stdSyncData;

struct SyncData
{
    SyncData()
            : agentID(nullptr)
            , ack(false)
            , conditionHolds(false)
            , transitionID(0)
    {
    }

    SyncData(const stdSyncData& s)
            : agentID(std::get<0>(s))
            , transitionID(std::get<1>(s))
            , conditionHolds(std::get<2>(s))
            , ack(std::get<3>(s))
    {
    }

    stdSyncData toStandard() const { return std::make_tuple(agentID, transitionID, conditionHolds, ack); }

    essentials::IdentifierConstPtr agentID;
    int64_t transitionID;
    bool conditionHolds;
    bool ack;
};

inline std::ostream& operator<<(std::ostream& o, const SyncData& sd)
{
    std::stringstream ss;
    ss << "SyncData: TransID " << sd.transitionID << " AgentID " << sd.agentID << (sd.conditionHolds ? "  COND" : " !COND") << (sd.ack ? "  ACK" : " !ACK")
       << std::endl;
    return o << ss.str();
}

} /* namespace alica */
