#pragma once

//#define AM_DEBUG

#include "../AlicaEngine.h"
#include "../IAlicaCommunication.h"
#include "../RunningPlan.h"
#include "../containers/AllocationAuthorityInfo.h"

#include <memory>
#include <mutex>
#include <vector>

namespace alica
{
class IAlicaCommunication;

/**
 * Manages communication wrt. conflict resolution.
 */
class AuthorityManager
{
public:
    AuthorityManager(AlicaEngine* ae);
    virtual ~AuthorityManager();
    void init();
    void close();
    void handleIncomingAuthorityMessage(const AllocationAuthorityInfo& aai);
    void tick(RunningPlan* p);
    void sendAllocation(const RunningPlan& p);

private:
    void processPlan(RunningPlan& p);
    bool authorityMatchesPlan(const AllocationAuthorityInfo& aai, const RunningPlan& p) const;

    std::vector<AllocationAuthorityInfo> _queue;
    const AlicaEngine* _engine;
    AgentIDConstPtr _localAgentID;
    std::mutex _mutex;
};
} /* namespace alica */
