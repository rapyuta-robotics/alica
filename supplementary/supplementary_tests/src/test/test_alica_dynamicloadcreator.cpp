#include "supplementary_tests/DynamicBehaviourCreator.h"
#include "supplementary_tests/DynamicPlanCreator.h"
#include <test_supplementary.h>

#include "communication/AlicaRosCommunication.h"
#include <engine/BasicBehaviour.h>
#include <engine/BasicPlan.h>
#include <engine/modelmanagement/factories/BehaviourFactory.h>
#include <engine/modelmanagement/factories/PlanFactory.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

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
    BehaviourContext ctx{&wm, behaviourModel->getName(), behaviourModel, "/var/tmp/customers"};
    std::unique_ptr<BasicBehaviour> behaviour = creator->createBehaviour(10, ctx);

    ASSERT_EQ("acmebehaviour", behaviour->getName());
    behaviour.release();
}

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
    PlanContext ctx{&wm, planModel->getName(), planModel, "/var/tmp/customers"};
    std::unique_ptr<BasicPlan> plan = creator->createPlan(10, ctx);

    ASSERT_EQ("acmeplan", plan->getName());
    plan.release();
}

} // namespace
} // namespace alica
