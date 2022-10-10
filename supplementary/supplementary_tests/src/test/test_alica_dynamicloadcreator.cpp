#include "supplementary_tests/DynamicBehaviourCreator.h"
#include "supplementary_tests/DynamicConditionCreator.h"
#include "supplementary_tests/DynamicPlanCreator.h"
#include <test_supplementary.h>

#include "communication/AlicaRosCommunication.h"
#include <engine/BasicBehaviour.h>
#include <engine/BasicPlan.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/modelmanagement/factories/BehaviourFactory.h>
#include <engine/modelmanagement/factories/ConditionFactory.h>
#include <engine/modelmanagement/factories/PlanFactory.h>
#include <engine/modelmanagement/factories/RuntimeConditionFactory.h>

#include <cstdlib>
#include <filesystem>
#include <stdlib.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{
/*
TEST(ForceLoad, simple_behaviour_load)
{
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    YAML::Node node;
    try {
        node = YAML::LoadFile(path + "/etc/plans/behaviours/Acme.beh");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/plans/behaviours/Acme.beh" << std::endl;
        AlicaEngine::abort("MM: Could not parse behaviour file: ", badFile.msg);
    }

    // Load model
    Behaviour* behaviourModel;
    behaviourModel = BehaviourFactory::create(node);

    // Create behaviour form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicBehaviourCreator>();
    std::string libraryPathFromAlicaYaml = path + "/../../../../../../install/lib";
    if (!std::filesystem::exists(libraryPathFromAlicaYaml)) {
        libraryPathFromAlicaYaml = path + "/../../../../../../devel/lib";
        if (!std::filesystem::exists(libraryPathFromAlicaYaml)) {
            std::cerr << "Library path not found:" << libraryPathFromAlicaYaml << " pwd:" << std::filesystem::current_path().string() << std::endl;
            // return;
        }
    }

    BehaviourContext ctx{&wm, behaviourModel->getName(), behaviourModel, libraryPathFromAlicaYaml, nullptr};
    std::unique_ptr<BasicBehaviour> behaviour = creator->createBehaviour(10, ctx);

    ASSERT_EQ("acmebehaviour", behaviour->getName());
}
*/
TEST(ForceLoad, simple_behaviour_withROS_load)
{
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    YAML::Node node;
    try {
        node = YAML::LoadFile(path + "/etc/plans/behaviours/Acme.beh");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/plans/behaviours/Acme.beh" << std::endl;
        AlicaEngine::abort("MM: Could not parse behaviour file: ", badFile.msg);
    }

    // Load model
    Behaviour* behaviourModel;
    behaviourModel = BehaviourFactory::create(node);

    // Create behaviour form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicBehaviourCreator>();
    std::string rosPackagePath = path + "/../../../../../../install/share/";
    if (!std::filesystem::exists(rosPackagePath)) {
        rosPackagePath = path + "/../../../../../../devel/share/";
        if (!std::filesystem::exists(rosPackagePath)) {
            std::cerr << "Library path not found:" << rosPackagePath << std::endl;
            // return;
        }
    }
    rosPackagePath = "ROS_PACKAGE_PATH=" + rosPackagePath;
    char* env = &rosPackagePath[0];
    //putenv(env);
    BehaviourContext ctx{&wm, behaviourModel->getName(), behaviourModel, "", nullptr};
    std::unique_ptr<BasicBehaviour> behaviour = creator->createBehaviour(10, ctx);

    ASSERT_EQ("acmebehaviour", behaviour->getName());
}
/*
TEST(ForceLoad, simple_plan_load)
{
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    YAML::Node globalNode;
    try {
        globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/hairy/Alica.yaml" << std::endl;
        AlicaEngine::abort("MM: Could not parse global config file: ", badFile.msg);
    }

    YAML::Node node;
    try {
        node = YAML::LoadFile(path + "/etc/plans/Acme.pml");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/plans/Acme.pml" << std::endl;
        AlicaEngine::abort("MM: Could not parse plan file: ", badFile.msg);
    }
    // Load model
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel;
    planModel = PlanFactory::create(configChangeListener, node);

    // Create plan form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicPlanCreator>();
    std::string libraryPathFromAlicaYaml = path + "/../../../../../../install/lib";
    if (!std::filesystem::exists(libraryPathFromAlicaYaml)) {
        libraryPathFromAlicaYaml = path + "/../../../../../../devel/lib";
        if (!std::filesystem::exists(libraryPathFromAlicaYaml)) {
            std::cerr << "Library path not found:" << libraryPathFromAlicaYaml << std::endl;
            // return;
        }
    }
    PlanContext ctx{&wm, planModel->getName(), planModel, libraryPathFromAlicaYaml, nullptr};

    std::unique_ptr<BasicPlan> plan = creator->createPlan(10, ctx);
    ASSERT_EQ("acmeplan", plan->getName());
}

TEST(ForceLoad, simple_plan_withROS_load)
{
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    YAML::Node globalNode;
    try {
        globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/hairy/Alica.yaml" << std::endl;
        AlicaEngine::abort("MM: Could not parse global config file: ", badFile.msg);
    }

    YAML::Node node;
    try {
        node = YAML::LoadFile(path + "/etc/plans/Acme.pml");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/plans/Acme.pml" << std::endl;
        AlicaEngine::abort("MM: Could not parse plan file: ", badFile.msg);
    }
    // Load model
    ConfigChangeListener configChangeListener(globalNode);
    Plan* planModel;
    planModel = PlanFactory::create(configChangeListener, node);

    // Create behaviour form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicPlanCreator>();
    std::string rosPackagePath = path + "/../../../../../../install/share/";
    if (!std::filesystem::exists(rosPackagePath)) {
        rosPackagePath = path + "/../../../../../../devel/share/";
        if (!std::filesystem::exists(rosPackagePath)) {
            std::cerr << "Library path not found:" << rosPackagePath << std::endl;
            // return;
        }
    }
    rosPackagePath = "ROS_PACKAGE_PATH=" + rosPackagePath;
    char* env = &rosPackagePath[0];
    putenv(env);
    PlanContext ctx{&wm, planModel->getName(), planModel, "", nullptr};

    std::unique_ptr<BasicPlan> plan = creator->createPlan(10, ctx);
    ASSERT_EQ("acmeplan", plan->getName());
}

TEST(ForceLoad, simple_condition_load)
{
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    YAML::Node globalNode;
    try {
        globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/hairy/Alica.yaml" << std::endl;
        AlicaEngine::abort("MM: Could not parse global config file: ", badFile.msg);
    }

    YAML::Node node;
    try {
        node = YAML::LoadFile(path + "/etc/plans/conditions/AcmeRuntimeCondition.cnd");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/plans/conditions/AcmeRuntimeCondition.cnd" << std::endl;
        AlicaEngine::abort("MM: Could not parse conditions file: ", badFile.msg);
    }
    // Load model
    ConfigChangeListener configChangeListener(globalNode);

    RuntimeCondition* conditionModel = RuntimeConditionFactory::create(node, nullptr);

    // Create condition form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicConditionCreator>();

    std::string libraryPathFromAlicaYaml = path + "/../../../../../../install/lib";
    if (!std::filesystem::exists(libraryPathFromAlicaYaml)) {
        libraryPathFromAlicaYaml = path + "/../../../../../../devel/lib";
        if (!std::filesystem::exists(libraryPathFromAlicaYaml)) {
            std::cerr << "Library path not found:" << libraryPathFromAlicaYaml << std::endl;
            // return;
        }
    }

    ConditionContext ctx{conditionModel->getName(), libraryPathFromAlicaYaml, conditionModel->getLibraryName(), 0};

    std::shared_ptr<BasicCondition> condition1 = creator->createConditions(ctx);
    std::shared_ptr<BasicCondition> condition2 = creator->createConditions(ctx);
    ASSERT_EQ(true, condition1->evaluate(nullptr, nullptr));
    ASSERT_EQ(true, condition2->evaluate(nullptr, nullptr));
}

TEST(ForceLoad, simple_condition_withROS_load)
{
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    YAML::Node globalNode;
    try {
        globalNode = YAML::LoadFile(path + "/etc/hairy/Alica.yaml");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/hairy/Alica.yaml" << std::endl;
        AlicaEngine::abort("MM: Could not parse global config file: ", badFile.msg);
    }

    YAML::Node node;
    try {
        node = YAML::LoadFile(path + "/etc/plans/conditions/AcmeRuntimeCondition.cnd");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/plans/conditions/AcmeRuntimeCondition.cnd" << std::endl;
        AlicaEngine::abort("MM: Could not parse conditions file: ", badFile.msg);
    }
    // Load model
    ConfigChangeListener configChangeListener(globalNode);

    RuntimeCondition* conditionModel = RuntimeConditionFactory::create(node, nullptr);

    // Create condition form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicConditionCreator>();
    std::string rosPackagePath = path + "/../../../../../../install/share/";
    if (!std::filesystem::exists(rosPackagePath)) {
        rosPackagePath = path + "/../../../../../../devel/share/";
        if (!std::filesystem::exists(rosPackagePath)) {
            std::cerr << "Library path not found:" << rosPackagePath << std::endl;
            // return;
        }
    }
    rosPackagePath = "ROS_PACKAGE_PATH=" + rosPackagePath;
    char* env = &rosPackagePath[0];
    putenv(env);

    ConditionContext ctx{conditionModel->getName(), "", conditionModel->getLibraryName(), 0};

    std::shared_ptr<BasicCondition> condition1 = creator->createConditions(ctx);
    std::shared_ptr<BasicCondition> condition2 = creator->createConditions(ctx);
    ASSERT_EQ(true, condition1->evaluate(nullptr, nullptr));
    ASSERT_EQ(true, condition2->evaluate(nullptr, nullptr));
}
*/
} // namespace
} // namespace alica
