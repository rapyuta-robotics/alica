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
    void handleIncomingAuthorityMessage(std::shared_ptr<AllocationAuthorityInfo> aai);
    void tick(RunningPlan& p);
    void sendAllocation(RunningPlan& p);

  private:
    std::vector<std::shared_ptr<AllocationAuthorityInfo>> queue;
    const AlicaEngine* engine;
    const supplementary::AgentID* localAgentID;
    std::mutex mu;
    void processPlan(RunningPlan& p);
    bool authorityMatchesPlan(const AllocationAuthorityInfo& aai, const RunningPlan& p) const;
};
} /* namespace alica */
