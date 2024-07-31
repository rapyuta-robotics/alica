#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicLoadingUtils.h>
#include <DynamicPlanCreator.h>
#include <DynamicTransitionConditionCreator.h>
#include <test_supplementary.h>

#include "communication/AlicaRosCommunication.h"
#include "engine/DefaultUtilityFunction.h"
#include <engine/BasicBehaviour.h>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/blackboard/Blackboard.h>
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
    AlicaDynamicLoading() { AlicaLogger::set<alica::AlicaDefaultLogger>(Verbosity::DEBUG, "TEST"); }

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

    std::shared_ptr<Blackboard> getGlobalBlackboard() { return std::make_shared<Blackboard>(); }

private:
    std::shared_ptr<Blackboard> _globalBlackboard;
};

TEST_F(AlicaDynamicLoading, simple_behaviour_load)
{
    std::string path = getRootPath();

    YAML::Node node;
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/behaviours/AcmeBeh.beh"));

    // Load model
    Behaviour* behaviourModel = BehaviourFactory::create(node);

    // Create behaviour from dll
    auto creator = std::make_unique<alica::DynamicBehaviourCreator>();
    BehaviourContext ctx{getGlobalBlackboard(), behaviourModel->getName(), behaviourModel, nullptr};
    std::unique_ptr<BasicBehaviour> behaviour;
    ASSERT_NO_THROW(behaviour = creator->createBehaviour(behaviourModel->getId(), ctx));

    // verify
    ASSERT_NE(behaviour, nullptr);
    ASSERT_EQ("AcmeBeh", behaviour->getName());
}

TEST_F(AlicaDynamicLoading, beh_symbol_not_found)
{
    std::string path = getRootPath();

    YAML::Node node;
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/behaviours/AcmeBeh.beh"));

    // ovewrite behaviour name to a value that we know doesn't exist
    node["name"] = "not_found";

    // Load model & verify if name & impl name have been overridden
    Behaviour* behaviourModel = BehaviourFactory::create(node);
    ASSERT_EQ(behaviourModel->getName(), "not_found");
    ASSERT_EQ(behaviourModel->getImplementationName(), "not_found");

    // should throw when we try to create the behaviour
    auto creator = std::make_unique<alica::DynamicBehaviourCreator>();
    BehaviourContext ctx{getGlobalBlackboard(), behaviourModel->getName(), behaviourModel, nullptr};
    ASSERT_THROW(creator->createBehaviour(behaviourModel->getId(), ctx), DynamicLoadingException);
}

TEST_F(AlicaDynamicLoading, simple_plan_load)
{
    std::string path = getRootPath();

    YAML::Node node, globalNode;
    ASSERT_NO_THROW(globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml"));
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/plans/AcmePlan.pml"));

    // Load model
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel = PlanFactory::create(configChangeListener, node);

    // Create plan from dll
    auto creator = std::make_unique<alica::DynamicPlanCreator>();
    PlanContext ctx{getGlobalBlackboard(), planModel->getName(), planModel, nullptr};
    std::unique_ptr<BasicPlan> plan;
    ASSERT_NO_THROW(plan = creator->createPlan(planModel->getId(), ctx));

    // verify
    ASSERT_NE(plan, nullptr);
    ASSERT_EQ("AcmePlan", plan->getName());
}

TEST_F(AlicaDynamicLoading, plan_symbol_not_found)
{
    std::string path = getRootPath();
    YAML::Node node;
    YAML::Node globalNode;

    ASSERT_NO_THROW(globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml"));
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/plans/AcmePlan.pml"));

    // ovewrite plan name to a value that we know doesn't exist
    node["name"] = "not_found";

    // Load model & verify if name & impl name have been overridden
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel = PlanFactory::create(configChangeListener, node);
    ASSERT_EQ(planModel->getName(), "not_found");
    ASSERT_EQ(planModel->getImplementationName(), "not_found");

    // should throw when we try to create the plan
    auto planCreator = std::make_unique<alica::DynamicPlanCreator>();
    PlanContext ctx{getGlobalBlackboard(), planModel->getName(), planModel, nullptr};
    ASSERT_THROW(planCreator->createPlan(planModel->getId(), ctx), DynamicLoadingException);
}

TEST_F(AlicaDynamicLoading, simple_condition_load)
{
    std::string path = getRootPath();

    YAML::Node node, globalNode;
    ASSERT_NO_THROW(globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml"));
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/plans/AcmePlan.pml"));

    // Load model
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel = PlanFactory::create(configChangeListener, node);

    ASSERT_NO_THROW(node = node["runtimeCondition"]);

    // Load model
    RuntimeCondition* conditionModel = RuntimeConditionFactory::create(node, planModel);

    // Create condition from dll
    auto creator = std::make_unique<alica::DynamicConditionCreator>();
    ConditionContext ctx{conditionModel->getName(), conditionModel->getLibraryName(), conditionModel->getId()};
    std::shared_ptr<BasicCondition> condition;
    ASSERT_NO_THROW(condition = creator->createConditions(conditionModel->getId(), ctx));

    // verify
    ASSERT_NE(condition, nullptr);
    ASSERT_EQ(true, condition->evaluate(nullptr, nullptr));
}

TEST_F(AlicaDynamicLoading, condition_symbol_not_found)
{
    std::string path = getRootPath();

    YAML::Node node, globalNode;
    ASSERT_NO_THROW(globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml"));
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/plans/AcmePlan.pml"));

    // Load model
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel = PlanFactory::create(configChangeListener, node);

    // ovewrite condition name to a value that we know doesn't exist
    node["name"] = "not_found";

    // Load model & verify if name has been overridden
    RuntimeCondition* conditionModel = RuntimeConditionFactory::create(node, planModel);
    ASSERT_EQ(conditionModel->getName(), "not_found");

    // should throw when we try to create the condition
    auto creator = std::make_unique<alica::DynamicConditionCreator>();
    ConditionContext ctx{conditionModel->getName(), conditionModel->getLibraryName(), conditionModel->getId()};
    ASSERT_THROW(creator->createConditions(conditionModel->getId(), ctx), DynamicLoadingException);
}

TEST_F(AlicaDynamicLoading, simple_transition_condition_load)
{
    std::string path = getRootPath();

    YAML::Node node;
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/conditions/ConditionRepository.cnd"));
    TransitionCondition* conditionModel;

    for (size_t i = 0; i < node["conditions"].size(); i++) {
        YAML::Node condition = node["conditions"][i];
        // Load model
        conditionModel = TransitionConditionFactory::create(condition, nullptr);
        if (conditionModel->getName() == "VariableHandlingStart") {
            break;
        }
    }

    ASSERT_EQ(conditionModel->getName(), "VariableHandlingStart");
    TransitionConditionContext ctx{conditionModel->getName(), conditionModel->getLibraryName(), conditionModel->getId()};

    // Create condition from dll
    std::unique_ptr<BlackboardBlueprint> testGlobalBlackboardBlueprint = std::make_unique<BlackboardBlueprint>();
    testGlobalBlackboardBlueprint->addKey("vhStartCondition", "bool", "protected");
    std::unique_ptr<Blackboard> testGlobalBlackboard = std::make_unique<Blackboard>(testGlobalBlackboardBlueprint.get());
    testGlobalBlackboard->impl().set("vhStartCondition", false);

    auto creator = std::make_unique<alica::DynamicTransitionConditionCreator>();
    TransitionConditionCallback transitionCondition;
    ASSERT_NO_THROW(transitionCondition = creator->createConditions(conditionModel->getId(), ctx));
    bool res = transitionCondition(nullptr, nullptr, testGlobalBlackboard.get());
    ASSERT_EQ(false, res);
}

TEST_F(AlicaDynamicLoading, transition_condition_symbol_not_found)
{
    std::string path = getRootPath();

    YAML::Node node;
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/conditions/ConditionRepository.cnd"));
    ASSERT_GT(node["conditions"].size(), 0);

    // ovewrite condition name to a value that we know doesn't exist
    node["conditions"][0]["name"] = "not_found";

    // Load model & verify if name has been overridden
    TransitionCondition* conditionModel = TransitionConditionFactory::create(node["conditions"][0], nullptr);
    ASSERT_EQ(conditionModel->getName(), "not_found");

    // should throw when we try to create the condition
    auto creator = std::make_unique<alica::DynamicTransitionConditionCreator>();
    TransitionConditionContext ctx{conditionModel->getName(), conditionModel->getLibraryName(), conditionModel->getId()};
    ASSERT_THROW(creator->createConditions(conditionModel->getId(), ctx), DynamicLoadingException);
}

TEST_F(AlicaDynamicLoading, simple_utility_function_load)
{
    std::string path = getRootPath();

    YAML::Node node, globalNode;
    ASSERT_NO_THROW(globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml"));
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/plans/AcmePlan.pml"));

    // Load model
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel = PlanFactory::create(configChangeListener, node);

    // Create utility function from dll. Note: utility function uses the same context values as the plan's context
    auto creator = std::make_unique<alica::DynamicUtilityFunctionCreator>();
    UtilityFunctionContext ctx{planModel->getName(), planModel->getLibraryName(), planModel->getId()};
    std::shared_ptr<BasicUtilityFunction> utilityFunction;
    ASSERT_NO_THROW(utilityFunction = creator->createUtility(planModel->getId(), ctx));

    // verify
    ASSERT_NE(utilityFunction, nullptr);
    ASSERT_TRUE(std::dynamic_pointer_cast<DefaultUtilityFunction>(utilityFunction->getUtilityFunction(planModel)));
}

TEST_F(AlicaDynamicLoading, utility_function_symbol_not_found)
{
    std::string path = getRootPath();

    YAML::Node node, globalNode;
    ASSERT_NO_THROW(globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml"));
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/plans/AcmePlan.pml"));

    // ovewrite plan name to a value that we know doesn't exist
    node["name"] = "not_found";

    // Load model & verify if name & impl name have been overridden
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel = PlanFactory::create(configChangeListener, node);
    ASSERT_EQ(planModel->getName(), "not_found");
    ASSERT_EQ(planModel->getImplementationName(), "not_found");

    // should throw when we try to create the utility function
    auto creator = std::make_unique<alica::DynamicUtilityFunctionCreator>();
    UtilityFunctionContext ctx{planModel->getName(), planModel->getLibraryName(), planModel->getId()};
    ASSERT_THROW(creator->createUtility(planModel->getId(), ctx), DynamicLoadingException);
}

TEST_F(AlicaDynamicLoading, simple_constraint_load)
{
    std::string path = getRootPath();

    YAML::Node node, globalNode;
    ASSERT_NO_THROW(globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml"));
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/plans/AcmePlan.pml"));

    // Load model
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel = PlanFactory::create(configChangeListener, node);

    ASSERT_NO_THROW(node = node["runtimeCondition"]);

    // Load model
    RuntimeCondition* conditionModel = RuntimeConditionFactory::create(node, planModel);

    // Create constraint from dll. Note: constraint uses the same context values as the condition's context
    auto creator = std::make_unique<alica::DynamicConstraintCreator>();
    ConstraintContext ctx{conditionModel->getName(), conditionModel->getLibraryName(), conditionModel->getId()};
    std::shared_ptr<BasicConstraint> constraint;
    ASSERT_NO_THROW(constraint = creator->createConstraint(conditionModel->getId(), ctx));

    // verify
    ASSERT_NE(constraint, nullptr);
}

TEST_F(AlicaDynamicLoading, constraint_symbol_not_found)
{
    std::string path = getRootPath();

    YAML::Node node, globalNode;
    ASSERT_NO_THROW(globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml"));
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/plans/AcmePlan.pml"));

    // Load model
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel = PlanFactory::create(configChangeListener, node);

    // ovewrite condition name to a value that we know doesn't exist
    node["name"] = "not_found";

    // Load model & verify if name has been overridden
    RuntimeCondition* conditionModel = RuntimeConditionFactory::create(node, planModel);
    ASSERT_EQ(conditionModel->getName(), "not_found");

    // should throw when we try to create the condition
    auto creator = std::make_unique<alica::DynamicConstraintCreator>();
    ConstraintContext ctx{conditionModel->getName(), conditionModel->getLibraryName(), conditionModel->getId()};
    ASSERT_THROW(creator->createConstraint(conditionModel->getId(), ctx), DynamicLoadingException);
}

TEST_F(AlicaDynamicLoading, beh_implementation_name_usage)
{
    std::string path = getRootPath();

    YAML::Node node;
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/behaviours/AcmeBeh.beh"));

    // set implementation name to the name of the behaviour & set the name of the behaviour to one that does not exist
    node["implementationName"] =
            node["name"]
                    .as<std::string>(); // need to use as<>(), otherwise node will be copied by reference & both will end up having "not_found" as their value
    node["name"] = "not_found";

    // Load model & verify if name & impl name have been overridden
    Behaviour* behaviourModel;
    behaviourModel = BehaviourFactory::create(node);
    ASSERT_EQ(behaviourModel->getName(), "not_found");
    ASSERT_EQ(behaviourModel->getImplementationName(), "AcmeBeh");

    // Create behaviour from dll
    auto creator = std::make_unique<alica::DynamicBehaviourCreator>();
    BehaviourContext ctx{getGlobalBlackboard(), behaviourModel->getName(), behaviourModel, nullptr};
    std::unique_ptr<BasicBehaviour> behaviour;
    ASSERT_NO_THROW(behaviour = creator->createBehaviour(behaviourModel->getId(), ctx));

    // verify
    ASSERT_NE(behaviour, nullptr);
    ASSERT_EQ(behaviourModel->getId(), behaviour->getId());
}

TEST_F(AlicaDynamicLoading, plan_implementation_name_usage)
{
    std::string path = getRootPath();

    YAML::Node node, globalNode;
    ASSERT_NO_THROW(globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml"));
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/plans/AcmePlan.pml"));

    // set implementation name to the name of the plan & set the name of the plan to one that does not exist
    node["implementationName"] = node["name"].as<std::string>();
    node["name"] = "not_found";

    // Load model & verify if name & impl name have been overridden
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel = PlanFactory::create(configChangeListener, node);
    ASSERT_EQ(planModel->getName(), "not_found");
    ASSERT_EQ(planModel->getImplementationName(), "AcmePlan");

    // Create plan from dll
    auto creator = std::make_unique<alica::DynamicPlanCreator>();
    PlanContext ctx{getGlobalBlackboard(), planModel->getName(), planModel, nullptr};
    std::unique_ptr<BasicPlan> plan;
    ASSERT_NO_THROW(plan = creator->createPlan(planModel->getId(), ctx));

    // verify
    ASSERT_NE(plan, nullptr);
    ASSERT_EQ(planModel->getId(), plan->getId());
}

// TODO: add a test to check if implementation name is used for creating utility functions. This will
// require starting the engine because the utility function is created at engine init time. There is
// no simple way to do this here

TEST_F(AlicaDynamicLoading, library_not_found)
{
    std::string path = getRootPath();

    YAML::Node node;
    ASSERT_NO_THROW(node = YAML::LoadFile(path + "/etc/behaviours/AcmeBeh.beh"));

    // ovewrite library name to a value that we know doesn't exist
    node["libraryName"] = "not_found";

    // Load model & verify if library name has been overriden
    Behaviour* behaviourModel = BehaviourFactory::create(node);
    ASSERT_EQ(behaviourModel->getLibraryName(), "not_found");

    // should throw when we try to load the behaviour
    auto creator = std::make_unique<alica::DynamicBehaviourCreator>();
    BehaviourContext ctx{getGlobalBlackboard(), behaviourModel->getName(), behaviourModel, nullptr};
    ASSERT_THROW(creator->createBehaviour(behaviourModel->getId(), ctx), DynamicLoadingException);
}

} // namespace
} // namespace alica
