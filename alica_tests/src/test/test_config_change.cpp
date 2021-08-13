#include <gtest/gtest.h>

#include "test_alica.h"

#include "engine/planselector/PartialAssignment.h"
namespace alica
{
namespace
{
class AlicaNotInitialized : public AlicaTestNotInitializedFixture
{
protected:
    const char* getMasterPlanName() const override { return "MasterPlan"; }
};

TEST_F(AlicaNotInitialized, TestUpdatingComponents)
{
    alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
            std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>());

    EXPECT_TRUE(ac->setOption<bool>("Alica.SilentStart", false));
    EXPECT_TRUE(ae->maySendMessages());

    EXPECT_TRUE(ac->setOption<bool>("Alica.SilentStart", true));
    EXPECT_FALSE(ae->maySendMessages());

    EXPECT_TRUE(ac->setOption<bool>("Alica.AllowIdling", false));
    EXPECT_FALSE(PartialAssignment::isIdlingAllowed());

    EXPECT_TRUE(ac->setOption<bool>("Alica.AllowIdling", true));
    EXPECT_TRUE(PartialAssignment::isIdlingAllowed());

    EXPECT_TRUE(!ac->init(creators));
}

TEST_F(AlicaNotInitialized, TestBlockConfigUpdatesAfterInitialization)
{
    alica::AlicaCreators creators(std::make_unique<alica::ConditionCreator>(), std::make_unique<alica::UtilityFunctionCreator>(),
            std::make_unique<alica::ConstraintCreator>(), std::make_unique<alica::BehaviourCreator>(), std::make_unique<alica::PlanCreator>());

    ac->setOption<int>("Alica.TeamTimeOut", 1000);
    EXPECT_TRUE(!ac->init(creators));

    // changes to config not allowed after initialization of AlicaContext
    ASSERT_FALSE(ac->setOption<int>("Alica.TeamTimeOut", 2000));
    EXPECT_EQ(1000, ac->getConfig()["Alica"]["TeamTimeOut"].as<int>());
}

TEST_F(AlicaNotInitialized, TestConfigUpdatesWithVector)
{
    // Set values to change later
    EXPECT_TRUE(ac->setOption<bool>("Local.IsGoalie", false));
    EXPECT_FALSE(ac->getConfig()["Local"]["IsGoalie"].as<bool>());

    EXPECT_TRUE(ac->setOption<float>("Local.AverageTranslation", 2000.0f));
    EXPECT_EQ(2000.0f, ac->getConfig()["Local"]["AverageTranslation"].as<float>());

    // Set multiple config values with vector
    std::pair<std::string, std::string> firstValue = std::make_pair("Local.IsGoalie", "true");
    std::pair<std::string, std::string> secondValue = std::make_pair("Local.AverageTranslation", "1000.0");

    std::vector<std::pair<std::string, std::string>> keyValuePairs;
    keyValuePairs.push_back(firstValue);
    keyValuePairs.push_back(secondValue);

    EXPECT_TRUE(ac->setOptions<std::string>(keyValuePairs));

    EXPECT_TRUE(ac->getConfig()["Local"]["IsGoalie"].as<bool>());
    EXPECT_EQ(1000.0f, ac->getConfig()["Local"]["AverageTranslation"].as<float>());
}
} // namespace
} // namespace alica
