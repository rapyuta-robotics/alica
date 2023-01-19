#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicLoadingUtils.h>
#include <DynamicPlanCreator.h>
#include <DynamicTransitionConditionCreator.h>
#include <test_supplementary.h>

#include "communication/AlicaRosCommunication.h"
#include <engine/BasicBehaviour.h>
#include <engine/BasicPlan.h>
#include <engine/logging/AlicaDefaultLogger.h>
#include <engine/logging/Logging.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/model/Transition.h>
#include <engine/model/TransitionCondition.h>
#include <engine/modelmanagement/factories/BehaviourFactory.h>
#include <engine/modelmanagement/factories/ConditionFactory.h>
#include <engine/modelmanagement/factories/PlanFactory.h>
#include <engine/modelmanagement/factories/RuntimeConditionFactory.h>
#include <engine/modelmanagement/factories/TransitionConditionFactory.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdlib.h>

#include <gtest/gtest.h>

namespace alica
{

namespace
{

class AlicaDynamicLoading : public ::testing::Test
{
public:
    AlicaDynamicLoading() { AlicaLogger::create<alica::AlicaDefaultLogger>(Verbosity::DEBUG, "TEST"); }

    std::string getRootPath() const
    {
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        return path;
    }

    std::string getLibRootPath() const
    {
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/dynamicLibRootPath", path, ".");
        return path;
    }
};

TEST_F(AlicaDynamicLoading, simple_behaviour_load)
{
    std::string libPath = getLibRootPath();

    YAML::Node node;
    ASSERT_NO_THROW((node = YAML::LoadFile(libPath + "/etc/plans/behaviours/AcmeBeh.beh")));

    // Load model
    Behaviour* behaviourModel;
    behaviourModel = BehaviourFactory::create(node);

    // Create behaviour form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicBehaviourCreator>();

    BehaviourContext ctx{&wm, behaviourModel->getName(), behaviourModel, nullptr};
    std::unique_ptr<BasicBehaviour> behaviour = creator->createBehaviour(10, ctx);

    ASSERT_EQ("AcmeBeh", behaviour->getName());
}

TEST_F(AlicaDynamicLoading, simple_plan_load)
{
    std::string path = getRootPath();
    std::string libPath = getLibRootPath();

    YAML::Node node, globalNode;
    ASSERT_NO_THROW((globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml")));
    ASSERT_NO_THROW((node = YAML::LoadFile(libPath + "/etc/plans/AcmePlan.pml")));

    // Load model
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel;
    planModel = PlanFactory::create(configChangeListener, node);

    // Create behaviour form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicPlanCreator>();

    PlanContext ctx{&wm, planModel->getName(), planModel, nullptr};

    std::unique_ptr<BasicPlan> plan = creator->createPlan(10, ctx);
    ASSERT_EQ("AcmePlan", plan->getName());
}

TEST_F(AlicaDynamicLoading, simple_condition_load)
{
    std::string libPath = getLibRootPath();

    YAML::Node node;
    ASSERT_NO_THROW((node = YAML::LoadFile(libPath + "/etc/plans/AcmePlan.pml")));
    ASSERT_NO_THROW((node = node["runtimeCondition"]));

    // Load model
    RuntimeCondition* conditionModel = RuntimeConditionFactory::create(node, nullptr);

    // Create condition form dll
    auto creator = std::make_unique<alica::DynamicConditionCreator>();

    ASSERT_EQ(conditionModel->getName(), "AcmeRuntimeCondition");
    ConditionContext ctx{conditionModel->getName(), conditionModel->getLibraryName(), 0};

    std::shared_ptr<BasicCondition> condition1 = creator->createConditions(1, ctx);
    std::shared_ptr<BasicCondition> condition2 = creator->createConditions(2, ctx);
    ASSERT_EQ(true, condition1->evaluate(nullptr, nullptr));
    ASSERT_EQ(true, condition2->evaluate(nullptr, nullptr));
}

TEST_F(AlicaDynamicLoading, simple_transition_condition_load)
{
    std::string libPath = getLibRootPath();

    YAML::Node node;
    ASSERT_NO_THROW((node = YAML::LoadFile(libPath + "/etc/plans/conditions/ConditionRepository.cnd")));
    ASSERT_NO_THROW((node = node["conditions"][0]));
    // Load model
    TransitionCondition* conditionModel = TransitionConditionFactory::create(node, nullptr);

    // Create condition form dll
    auto creator = std::make_unique<alica::DynamicTransitionConditionCreator>();

    ASSERT_EQ(conditionModel->getName(), "VariableHandlingStart");
    TransitionConditionContext ctx{conditionModel->getName(), conditionModel->getLibraryName(), 0, 0};

    auto transitionCondition = creator->createConditions(1, ctx);
    bool res = transitionCondition(nullptr, nullptr, nullptr);
    ASSERT_EQ(false, res);
}

TEST_F(AlicaDynamicLoading, simple_waitbehaviour_load)
{
    std::string libPath = getLibRootPath();

    YAML::Node node;
    ASSERT_NO_THROW((node = YAML::LoadFile(libPath + "/etc/plans/behaviours/WaitBehaviour.beh")));

    // Load model
    Behaviour* behaviourModel;
    behaviourModel = BehaviourFactory::create(node);

    // Create behaviour form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicBehaviourCreator>();

    BehaviourContext ctx{&wm, behaviourModel->getName(), behaviourModel, nullptr};
    std::unique_ptr<BasicBehaviour> behaviour = creator->createBehaviour(10, ctx);

    ASSERT_EQ("WaitBehaviour", behaviour->getName());
}

} // namespace
} // namespace alica
