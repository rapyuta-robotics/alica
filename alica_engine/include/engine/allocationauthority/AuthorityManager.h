#pragma once

//#define AM_DEBUG

#include "../IAlicaCommunication.h"
#include "../RunningPlan.h"
#include "../containers/AllocationAuthorityInfo.h"

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
    AuthorityManager(AlicaEngine* ae);
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
    AlicaEngine* _engine;
    essentials::IdentifierConstPtr _localAgentID;
    std::mutex _mutex;
};
} /* namespace alica */
