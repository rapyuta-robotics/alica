#pragma once

#include <engine/AlicaEngine.h>
#include <engine/BasicBehaviour.h>

#include <memory>

namespace alica::test
{
class Util
{
public:
    static std::shared_ptr<BasicBehaviour> getBasicBehaviour(alica::AlicaEngine* ae, int64_t behaviourID, int64_t configurationID);
    static bool isStateActive(alica::AlicaEngine* ae, int64_t id);
    static bool isPlanActive(alica::AlicaEngine* ae, int64_t id);
    static const alica::Agent* getLocalAgent(alica::AlicaEngine* ae);
    static int getTeamSize(alica::AlicaEngine* ae);
    static const alica::Agent* getAgentByID(alica::AlicaEngine* ae, essentials::IdentifierConstPtr agentID);

private:
    static bool isPlanActiveHelper(const RunningPlan* rp, int64_t id);
    static bool isStateActiveHelper(const RunningPlan* rp, int64_t id);
};
} // namespace alica::test
