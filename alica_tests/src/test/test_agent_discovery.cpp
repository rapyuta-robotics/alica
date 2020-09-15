#include "TestWorldModel.h"
#include "test_alica.h"

#include <alica/test/Util.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/TeamObserver.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>
#include <engine/model/Task.h>
#include <engine/teammanager/TeamManager.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaEngineAgentDiscoveryTest : public AlicaTestMultiAgentFixture
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

    void verifyAgents(const alica::Agent* original, const alica::Agent* discovered)
    {
        ASSERT_NE(original, nullptr);
        ASSERT_NE(discovered, nullptr);
        ASSERT_EQ(discovered->getName(), original->getName());
        ASSERT_EQ(discovered->getSdk(), original->getSdk());
        ASSERT_EQ(discovered->getPlanHash(), original->getPlanHash());
        ASSERT_EQ(*(discovered->getId()), *(original->getId()));
    }
};

TEST_F(AlicaEngineAgentDiscoveryTest, AgentDiscovered)
{
    ASSERT_NO_SIGNAL
    ASSERT_EQ(1, alica::test::Util::getTeamSize(aes[0]));
    ASSERT_EQ(1, alica::test::Util::getTeamSize(aes[1]));
    aes[0]->start();
    aes[1]->start();
    // Let agent announce their presence
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    // Process presence announcement
    acs[0]->stepEngine();
    acs[1]->stepEngine();
    ASSERT_EQ(2, alica::test::Util::getTeamSize(aes[0]));
    ASSERT_EQ(2, alica::test::Util::getTeamSize(aes[1]));

    uint64_t id = 8;
    const alica::Agent* hairydiscovered = alica::test::Util::getAgentByID(aes[0], acs[0]->getID(id));
    const alica::Agent* hairyoriginal = alica::test::Util::getLocalAgent(aes[1]);
    verifyAgents(hairydiscovered, hairyoriginal);

    id = 9;
    const alica::Agent* nasediscovered = alica::test::Util::getAgentByID(aes[1], acs[1]->getID(id));
    const alica::Agent* naseoriginal = alica::test::Util::getLocalAgent(aes[0]);
    verifyAgents(nasediscovered, naseoriginal);

    // Reject agent with mismatching plan hash
    id = 11;
    alica::AgentAnnouncement aa;
    aa.planHash = acs[0]->getVersion() + 1;
    aa.senderSdk = acs[0]->getVersion();
    aa.senderID = acs[0]->getID(id);
    aa.roleId = 1222973297047; // Attacker
    aa.senderName = "myo";
    aes[0]->editTeamManager().handleAgentAnnouncement(aa);
    acs[0]->stepEngine();
    const alica::Agent* myodiscovered = alica::test::Util::getAgentByID(aes[0], acs[0]->getID(id));
    ASSERT_EQ(myodiscovered, nullptr);
}
} // namespace
} // namespace alica