#include <gtest/gtest.h>

#include "test_alica.h"

#include "engine/planselector/PartialAssignment.h"
#include <DynamicBehaviourCreator.h>
#include <engine/ConfigChangeListener.h>

namespace alica
{
namespace
{
class AlicaNotInitialized : public AlicaTestNotInitializedFixture
{
protected:
    const char* getMasterPlanName() const override { return "MasterPlan"; }
};

TEST_F(AlicaNotInitialized, TestUpdatingComponents_001)
{
    alica::AlicaCreators creators(std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
            std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
            std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>());

    EXPECT_TRUE(ac->setOption<bool>("Alica.SilentStart", false));
    EXPECT_EQ(0, ac->init(std::move(creators), true));
    ae = (AlicaTestsEngineGetter::getEngine(ac.get()));

    EXPECT_TRUE(ae->maySendMessages());
}
TEST_F(AlicaNotInitialized, TestUpdatingComponents_002)
{
    alica::AlicaCreators creators(std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
            std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
            std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>());

    EXPECT_TRUE(ac->setOption<bool>("Alica.SilentStart", true));
    EXPECT_EQ(0, ac->init(std::move(creators), true));
    ae = AlicaTestsEngineGetter::getEngine(ac.get());

    EXPECT_FALSE(ae->maySendMessages());
}

TEST_F(AlicaNotInitialized, TestUpdatingComponents_003)
{
    EXPECT_TRUE(ac->setOption<bool>("Alica.AllowIdling", false));
    EXPECT_TRUE(PartialAssignment::isIdlingAllowed());

    // Some options can be set but become available only after init
    EXPECT_TRUE(ac->setOption<bool>("Alica.AllowIdling", true));
    EXPECT_TRUE(PartialAssignment::isIdlingAllowed());

    alica::AlicaCreators creators(std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
            std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
            std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>());
    EXPECT_EQ(0, ac->init(std::move(creators), true));
    ae = AlicaTestsEngineGetter::getEngine(ac.get());

    EXPECT_TRUE(PartialAssignment::isIdlingAllowed());
}

TEST_F(AlicaNotInitialized, TestBlockConfigUpdatesAfterInitialization)
{
    alica::AlicaCreators creators(std::make_unique<alica::DynamicConditionCreator>(), std::make_unique<alica::DynamicUtilityFunctionCreator>(),
            std::make_unique<alica::DynamicConstraintCreator>(), std::make_unique<alica::DynamicBehaviourCreator>(),
            std::make_unique<alica::DynamicPlanCreator>(), std::make_unique<alica::DynamicTransitionConditionCreator>());

    ac->setOption<int>("Alica.TeamTimeOut", 1000);
    EXPECT_EQ(0, ac->init(std::move(creators), true));

    // changes to config is now allowed after initialization of AlicaContext
    EXPECT_TRUE(ac->setOption<int>("Alica.TeamTimeOut", 2000));
    EXPECT_EQ(2000, ac->getConfig()["Alica"]["TeamTimeOut"].as<int>());
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

TEST(ConfigChangeListenerTest, TestConfigSubscribeAndUnsubscribe)
{
    // create new configChangeListener with empty config node
    YAML::Node cfg;
    alica::ConfigChangeListener ccl(cfg);
    bool called = false;

    std::function<void(const YAML::Node& cfg)> cb = [&](const YAML::Node& cfg) { called = true; };
    uint64_t cbId = ccl.subscribe(cb);
    ccl.reloadConfig(cfg);
    ASSERT_TRUE(called);

    ccl.unsubscribe(cbId);
    called = false;
    ccl.reloadConfig(cfg);
    ASSERT_FALSE(called);
}
} // namespace
} // namespace alica
