#pragma once

//#define AM_DEBUG

#include "../IAlicaCommunication.h"
#include "../RunningPlan.h"
#include "../containers/AllocationAuthorityInfo.h"
#include "engine/Types.h"

#include <memory>
#include <mutex>
#include <vector>

namespace alica
{
class IAlicaCommunication;
class AlicaEngine;
/**
 * Manages communication wrt. conflict resolution.
 */
class AuthorityManager
{
public:
    AuthorityManager(const YAML::Node& config, const IAlicaCommunication& communicator, const AlicaClock& clock, TeamManager& teamManager);
    ~AuthorityManager();
    void init();
    void close();
    void handleIncomingAuthorityMessage(const AllocationAuthorityInfo& aai);
    void tick(RunningPlan* p);
    void sendAllocation(const RunningPlan& p);

private:
    void processPlan(RunningPlan& p);
    bool authorityMatchesPlan(const AllocationAuthorityInfo& aai, const RunningPlan& p) const;

    std::vector<AllocationAuthorityInfo> _queue;
    const YAML::Node& _config;
    const IAlicaCommunication& _communicator;
    const AlicaClock& _clock;
    TeamManager& _tm;
    AgentId _localAgentID;
    std::mutex _mutex;
};
} /* namespace alica */
