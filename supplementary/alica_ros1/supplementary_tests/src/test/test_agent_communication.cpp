#include "test_supplementary.h"

#include <alica/test/Util.h>

namespace supplementary
{
namespace
{

class AlicaInactiveAgentCommunicationTest : public AlicaTestMultiAgentFixture
{
protected:
    const int agentCount = 2;
    const char* getRoleSetName() const override { return "RolesetTA"; }
    const char* getMasterPlanName() const override { return "VHMaster"; }
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "hairy";
        } else {
            return "nase";
        }
    }

    void SetUp() override
    {
        // determine the path to the test config
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("rootPath", path, ".");

        for (int i = 0; i < getAgentCount(); ++i) {
            alica::AlicaCreators creators = {std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
                    std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
                    std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>()};

            cbQueues.emplace_back(std::make_unique<ros::CallbackQueue>());
            spinners.emplace_back(std::make_unique<ros::AsyncSpinner>(4, cbQueues.back().get()));
            alica::AlicaContext* ac =
                    new alica::AlicaContext(AlicaContextParams(getHostName(i), {path + "/etc"}, getRoleSetName(), getMasterPlanName(), stepEngine()));
            ASSERT_TRUE(ac->isValid());
            ac->setCommunicator<alicaRosProxy::AlicaRosCommunication>(*cbQueues.back());
            ac->setTimerFactory<alicaRosTimer::AlicaRosTimerFactory>(*cbQueues.back());
            ac->setOption<int>("Alica.TeamTimeOut", -100);
            // ac->getConfig()["Alica"]["TeamTimeOut"]. = -100;
            EXPECT_EQ(0, ac->init(std::move(creators), true));
            alica::AlicaEngine* ae = AlicaTestsEngineGetter::getEngine(ac);
            const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();
            spinners.back()->start();
            acs.push_back(ac);
            aes.push_back(ae);
        }
    }

    bool stepEngine() const override { return true; }
};

TEST_F(AlicaInactiveAgentCommunicationTest, inactiveAgentTest)
{
    // Test if application crashes when agent is inactive and sends msgs which time out
    aes[0]->start();
    aes[1]->start();
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());

    STEP_ALL_UNTIL(acs, alica::test::Util::getTeamSize(aes[0]) != 2 || alica::test::Util::getTeamSize(aes[1]) != 2);
}
} // namespace
} // namespace supplementary
