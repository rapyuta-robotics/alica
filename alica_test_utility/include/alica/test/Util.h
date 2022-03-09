#pragma once

#include <engine/AlicaEngine.h>
#include <engine/BasicBehaviour.h>

#include <chrono>
#include <memory>

#define STEP_UNTIL(condition)                                                                                                                                  \
    do {                                                                                                                                                       \
        for (int i = 0; i < 10; ++i) {                                                                                                                         \
            ac->stepEngine();                                                                                                                                  \
            if (condition) {                                                                                                                                   \
                break;                                                                                                                                         \
            }                                                                                                                                                  \
            std::this_thread::sleep_for(std::chrono::milliseconds(10));                                                                                        \
        }                                                                                                                                                      \
    } while (0)

#define SLEEP_UNTIL(condition)                                                                                                                                 \
    do {                                                                                                                                                       \
        for (int i = 0; i < 100; ++i) {                                                                                                                        \
            if (condition) {                                                                                                                                   \
                break;                                                                                                                                         \
            }                                                                                                                                                  \
            std::this_thread::sleep_for(std::chrono::milliseconds(10));                                                                                        \
        }                                                                                                                                                      \
    } while (0)

namespace alica::test
{
class Util
{
public:
    static BasicBehaviour* getBasicBehaviour(alica::AlicaEngine* ae, int64_t behaviourID, int64_t configurationID);
    static BasicPlan* getBasicPlan(alica::AlicaEngine* ae, int64_t planId, int64_t configurationId);
    static bool hasPlanSucceeded(alica::AlicaEngine* ae, int64_t id);
    static bool isStateActive(alica::AlicaEngine* ae, int64_t id);
    static bool isPlanActive(alica::AlicaEngine* ae, int64_t id);
    static const alica::Agent* getLocalAgent(alica::AlicaEngine* ae);
    static int getTeamSize(alica::AlicaEngine* ae);
    static const alica::Agent* getAgentByID(alica::AlicaEngine* ae, AgentId agentID);

private:
    static BasicBehaviour* getBasicBehaviourHelper(const RunningPlan* rp, int64_t behaviourId);
    static BasicPlan* getBasicPlanHelper(const RunningPlan* rp, int64_t planId);
    static bool hasPlanSucceededHelper(const RunningPlan* rp, int64_t id);
    static bool isPlanActiveHelper(const RunningPlan* rp, int64_t id);
    static bool isStateActiveHelper(const RunningPlan* rp, int64_t id);
};
} // namespace alica::test
