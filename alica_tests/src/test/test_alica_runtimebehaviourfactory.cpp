#include "test_alica.h"

#include <alica/test/Util.h>
#include <alica_tests/BehaviourCreator.h>
#include <alica_tests/TestWorldModel.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaRuntimeBehaviour : public AlicaTestMultiAgentFixture
{
protected:
    const int agentCount = 2;
    const char* getRoleSetName() const override { return "RolesetTA"; }
    const char* getMasterPlanName() const override { return "MultiAgentTestMaster"; }
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "hairy";
        } else {
            return "nase";
        }
    }
};

TEST_F(AlicaRuntimeBehaviour, scheduling)
{
    RuntimeBehaviourFactory rtbf(std::make_unique<alica::BehaviourCreator>(), nullptr, nullptr);
}
} // namespace
} // namespace alica