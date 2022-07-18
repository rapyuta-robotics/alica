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
class IAlicaLogger;
/**
 * Manages communication wrt. conflict resolution.
 */
class AuthorityManager
{
public:
    AuthorityManager(ConfigChangeListener& configChangeListener, const IAlicaCommunication& communicator, const AlicaClock& clock, TeamManager& teamManager,
            IAlicaLogger& logger);
    ~AuthorityManager();
    void init();
    void close();
    void handleIncomingAuthorityMessage(const AllocationAuthorityInfo& aai);
    void tick(RunningPlan* p);
    void sendAllocation(const RunningPlan& p);

    void reload(const YAML::Node& config);

private:
    void processPlan(RunningPlan& p);
    bool authorityMatchesPlan(const AllocationAuthorityInfo& aai, const RunningPlan& p) const;

    std::vector<AllocationAuthorityInfo> _queue;
    ConfigChangeListener& _configChangeListener;
    const IAlicaCommunication& _communicator;
    const AlicaClock& _clock;
    TeamManager& _tm;
    AgentId _localAgentID;
    bool _maySendMessages;
    std::mutex _mutex;
    IAlicaLogger& _logger;
};
} /* namespace alica */
