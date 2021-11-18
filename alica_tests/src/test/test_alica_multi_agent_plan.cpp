#include "test_alica.h"

#include "Behaviour/Attack.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include <alica_tests/TestWorldModel.h>
#include "UtilityFunctionCreator.h"

#include <alica/test/Util.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/Assignment.h>
#include <engine/BasicBehaviour.h>
#include <engine/BehaviourPool.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/model/Behaviour.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaMultiAgent : public AlicaTestMultiAgentFixture
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
/**
 * Tests whether it is possible to use multiple agents.
 */
TEST_F(AlicaMultiAgent, runMultiAgentPlan)
{
    ASSERT_NO_SIGNAL
    aes[0]->start();
    aes[1]->start();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    acs[0]->stepEngine();
    acs[1]->stepEngine();

    for (int i = 0; i < 20; i++) {
        std::cout << "AE1 step " << i << "(" << acs[0]->getLocalAgentId() << ")" << std::endl;
        acs[0]->stepEngine();

        std::cout << "AE2 step " << i << "(" << acs[1]->getLocalAgentId() << ")" << std::endl;
        acs[1]->stepEngine();

        if (i < 10) {
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], 1413200842974));
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 1413200842974));
        }
        if (i == 10) {
            std::cout << "1--------- Initial State passed ---------" << std::endl;
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201227586(true);
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201227586(true);
        }
        if (i > 11 && i < 15) {
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], 1413201213955));
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 1413201213955));
            ASSERT_TRUE(alica::test::Util::isPlanActive(aes[0], 1413200862180));
            ASSERT_TRUE(alica::test::Util::isPlanActive(aes[1], 1413200862180));
        }
        if (i == 15) {
            ASSERT_GT(std::dynamic_pointer_cast<alica::Attack>(alica::test::Util::getBasicBehaviour(aes[0], 1402488848841, 0))->callCounter, 5);
            if (std::dynamic_pointer_cast<alica::Attack>(alica::test::Util::getBasicBehaviour(aes[0], 1402488848841, 0))->callCounter > 3) {
                alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201052549(true);
                alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201052549(true);
                alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201370590(true);
                alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201370590(true);
            }
            std::cout << "2--------- Engagement to cooperative plan passed ---------" << std::endl;
        }
        if (i == 16) {
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 1413201030936) || alica::test::Util::isStateActive(aes[0], 1413201030936))
                                        << std::endl
                                        << "TCS[1] " << (alica::test::Util::isStateActive(aes[1], 1413201030936) ? " 1413201030936 is active" : " 1413201030936 is inactive") << " "
                                        << "TCS[0] " << (alica::test::Util::isStateActive(aes[0], 1413201030936) ? " 1413201030936 is active" : " 1413201030936 is inactive") << std::endl;

            ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 1413807264574) || alica::test::Util::isStateActive(aes[0], 1413807264574))
                                        << std::endl
                                        << "TCS[1] " << (alica::test::Util::isStateActive(aes[1], 1413807264574) ? " 1413807264574 is active" : " 1413807264574 is inactive") << " "
                                        << "TCS[0] " << (alica::test::Util::isStateActive(aes[0], 1413807264574) ? " 1413807264574 is active" : " 1413807264574 is inactive") << std::endl;
            alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201227586(false);
            alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201227586(false);
            std::cout << "3--------- Passed transitions in subplan passed ---------" << std::endl;
        }
        if (i >= 17 && i <= 18) {
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 1413201030936) || alica::test::Util::isStateActive(aes[0], 1413201030936))
                                        << "TCS[0] State: " << (alica::test::Util::isStateActive(aes[0], 1413201030936) ? " 1413201030936 is active" : " 1413201030936 is inactive") << " "
                                        << "TCS[1] State: " << (alica::test::Util::isStateActive(aes[1], 1413201030936) ? " 1413201030936 is active" : " 1413201030936 is inactive") << std::endl;
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 1413807264574) || alica::test::Util::isStateActive(aes[0], 1413807264574))
                                        << "TCS[0] State: " << (alica::test::Util::isStateActive(aes[0], 1413807264574) ? " 1413807264574 is active" : " 1413807264574 is inactive") << " "
                                        << "TCS[1] State: " << (alica::test::Util::isStateActive(aes[1], 1413807264574) ? " 1413807264574 is active" : " 1413807264574 is inactive") << std::endl;
            if (i == 18) {
                std::cout << "4--------- Stayed in these state although previous transitions are not true anymore ---------" << std::endl;
                alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201389955(true);
                alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201389955(true);
            }
        }
        if (i == 19) {
            ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 1413201380359) && alica::test::Util::isStateActive(aes[0], 1413201380359))
                                        << "TCS[0] State: " << (alica::test::Util::isStateActive(aes[0], 1413201380359) ? " 1413201380359 is active" : " 1413201380359 is inactive") << " "
                                        << "TCS[1] State: " << (alica::test::Util::isStateActive(aes[1], 1413201380359) ? " 1413201380359 is active" : " 1413201380359 is inactive") << std::endl;
        }
    }
}
} // namespace
} // namespace alica