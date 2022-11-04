#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicLoadingUtils.h>
#include <DynamicPlanCreator.h>
#include <test_supplementary.h>

#include "communication/AlicaRosCommunication.h"
#include <engine/BasicBehaviour.h>
#include <engine/BasicPlan.h>
#include <engine/logging/AlicaDefaultLogger.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/modelmanagement/factories/BehaviourFactory.h>
#include <engine/modelmanagement/factories/ConditionFactory.h>
#include <engine/modelmanagement/factories/PlanFactory.h>
#include <engine/modelmanagement/factories/RuntimeConditionFactory.h>

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
    std::string _ldLibraryPath;

    AlicaDynamicLoading() { AlicaLogger::create<alica::AlicaDefaultLogger>(Verbosity::INFO, "TEST"); }

    void exportLdLibraryPath(const std::string& rootPath)
    {
        _ldLibraryPath = rootPath + "/../../../../../../install/lib/";
        _ldLibraryPath = simplifyPath(_ldLibraryPath);
        if (!std::filesystem::exists(_ldLibraryPath)) {
            _ldLibraryPath = rootPath + "/../../../../../../devel/lib/";
            if (!std::filesystem::exists(_ldLibraryPath)) {
                std::cerr << "Library path not found:" << _ldLibraryPath << std::endl;
            }
        }
        _ldLibraryPath = "LD_LIBRARY_PATH=" + _ldLibraryPath;
        char* env = &_ldLibraryPath[0];
        putenv(env);
    }

    std::string getRootPath() const
    {
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");
        return path;
    }
};

TEST_F(AlicaDynamicLoading, simple_behaviour_load)
{
    std::string path = getRootPath();

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

    exportLdLibraryPath(path);

    // Create behaviour form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicBehaviourCreator>();

    BehaviourContext ctx{&wm, behaviourModel->getName(), behaviourModel, nullptr};
    std::unique_ptr<BasicBehaviour> behaviour = creator->createBehaviour(10, ctx);

    ASSERT_EQ("acmebehaviour", behaviour->getName());
}

TEST_F(AlicaDynamicLoading, simple_plan_load)
{
    std::string path = getRootPath();
    exportLdLibraryPath(path);

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

    PlanContext ctx{&wm, planModel->getName(), planModel, nullptr};

    std::unique_ptr<BasicPlan> plan = creator->createPlan(10, ctx);
    ASSERT_EQ("acmeplan", plan->getName());
}

TEST_F(AlicaDynamicLoading, simple_condition_load)
{
    std::string path = getRootPath();
    exportLdLibraryPath(path);

    YAML::Node node;
    try {
        node = YAML::LoadFile(path + "/etc/plans/conditions/AcmeRuntimeCondition.cnd");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/plans/conditions/AcmeRuntimeCondition.cnd" << std::endl;
        AlicaEngine::abort("MM: Could not parse conditions file: ", badFile.msg);
    }

    // Load model
    RuntimeCondition* conditionModel = RuntimeConditionFactory::create(node, nullptr);

    // Create condition form dll
    auto creator = std::make_unique<alica::DynamicConditionCreator>();

    ConditionContext ctx{conditionModel->getName(), conditionModel->getLibraryName(), 0};

    std::shared_ptr<BasicCondition> condition1 = creator->createConditions(ctx);
    std::shared_ptr<BasicCondition> condition2 = creator->createConditions(ctx);
    ASSERT_EQ(true, condition1->evaluate(nullptr, nullptr));
    ASSERT_EQ(true, condition2->evaluate(nullptr, nullptr));
}

TEST_F(AlicaDynamicLoading, simple_waitbehaviour_load)
{
    std::string path = getRootPath();
    exportLdLibraryPath(path);

    YAML::Node node;
    try {
        node = YAML::LoadFile(path + "/etc/plans/behaviours/WaitBehaviour.beh");
    } catch (YAML::BadFile& badFile) {
        std::cerr << path + "/etc/plans/behaviours/WaitBehaviour.beh" << std::endl;
        AlicaEngine::abort("MM: Could not parse behaviour file: ", badFile.msg);
    }

    // Load model
    Behaviour* behaviourModel;
    behaviourModel = BehaviourFactory::create(node);

    // Create behaviour form dll
    IAlicaWorldModel wm;
    auto creator = std::make_unique<alica::DynamicBehaviourCreator>();

    BehaviourContext ctx{&wm, behaviourModel->getName(), behaviourModel, nullptr};
    std::unique_ptr<BasicBehaviour> behaviour = creator->createBehaviour(10, ctx);

    ASSERT_EQ("waitbehaviour", behaviour->getName());
    // behaviour->start(nullptr);
    behaviour->run(nullptr);
}

} // namespace
} // namespace alica
