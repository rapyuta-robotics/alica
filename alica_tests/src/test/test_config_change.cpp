#include <gtest/gtest.h>

#include "test_alica.h"
namespace alica
{
    class AlicaEngine;
    class AlicaContext;

    TEST(ConfigChangeListener, AlicaEngine)
{
    std::string p = "Alica.TeamTimeOut";

    // determine the path to the test config
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    AlicaContext *ac = new alica::AlicaContext();
    ac->setLocalAgentName("nase");
    ac->buildObjects("RoleSet", "MasterPlan", true, path + "/etc/Alica.yaml");

    ac->setOption<int>(p, 50);
    EXPECT_EQ(50, ac->getConfig()["Alica"]["TeamTimeOut"].as<int>());

    ASSERT_TRUE(ac->isValid());
    ac->setCommunicator<alicaDummyProxy::AlicaDummyCommunication>();
    alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
                                  std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>());
    AlicaEngine* ae = AlicaTestsEngineGetter::getEngine(ac);
    const_cast<IAlicaCommunication&>(ae->getCommunicator()).startCommunication();

    ac->setOption<int>(p, 1000);
    EXPECT_EQ(1000, ac->getConfig()["Alica"]["TeamTimeOut"].as<int>());

    EXPECT_TRUE(ae->init(creators));

    //changes to config not allowed after initialization of AlicaContext
    ac->setOption<int>(p, 2000);
    EXPECT_EQ(1000, ac->getConfig()["Alica"]["TeamTimeOut"].as<int>());

    ac->terminate();
    delete ac;


}
}

