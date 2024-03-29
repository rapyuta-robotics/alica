#pragma once

#include <engine/AlicaEngine.h>
#include <engine/BasicBehaviour.h>
#include <engine/BasicPlan.h>

#include <chrono>
#include <gtest/gtest.h>
#include <memory>

#define STEP_UNTIL1(condition)                                                                                                                                 \
    do {                                                                                                                                                       \
        for (int i = 0; i < 10; ++i) {                                                                                                                         \
            ac->stepEngine();                                                                                                                                  \
            if (condition) {                                                                                                                                   \
                break;                                                                                                                                         \
            }                                                                                                                                                  \
            std::this_thread::sleep_for(std::chrono::milliseconds(10));                                                                                        \
        }                                                                                                                                                      \
    } while (0)

#define STEP_UNTIL2(ac, condition)                                                                                                                             \
    do {                                                                                                                                                       \
        for (int i = 0; i < 10; ++i) {                                                                                                                         \
            (ac)->stepEngine();                                                                                                                                \
            if (condition) {                                                                                                                                   \
                break;                                                                                                                                         \
            }                                                                                                                                                  \
            std::this_thread::sleep_for(std::chrono::milliseconds(10));                                                                                        \
        }                                                                                                                                                      \
    } while (0)

#define GET_STEP_MACRO(_0, _1, _2, NAME, ...) NAME
#define STEP_UNTIL(...) GET_STEP_MACRO(_0, __VA_ARGS__, STEP_UNTIL2, STEP_UNTIL1)(__VA_ARGS__)

#define STEP_ALL_UNTIL(ac, condition)                                                                                                                          \
    do {                                                                                                                                                       \
        for (int i = 0; i < 50; ++i) {                                                                                                                         \
            for (auto& currentAc : (ac)) {                                                                                                                     \
                currentAc->stepEngine();                                                                                                                       \
            }                                                                                                                                                  \
            if (condition) {                                                                                                                                   \
                break;                                                                                                                                         \
            }                                                                                                                                                  \
        }                                                                                                                                                      \
        std::this_thread::sleep_for(std::chrono::milliseconds(10));                                                                                            \
    } while (0)

#define STEP_UNTIL_ASSERT_TRUE(ac, condition)                                                                                                                  \
    STEP_UNTIL(ac, condition);                                                                                                                                 \
    ASSERT_TRUE(condition)

#define STEP_UNTIL_ASSERT_EQ(ac, lhs, rhs)                                                                                                                     \
    STEP_UNTIL(ac, ((lhs) == (rhs)))                                                                                                                           \
    ASSERT_EQ(lhs, rhs)

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
    static BasicBehaviour* getBasicBehaviour(alica::AlicaEngine* ae, int64_t behaviourID);
    static BasicPlan* getBasicPlan(alica::AlicaEngine* ae, int64_t planId);
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
