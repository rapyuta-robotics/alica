#include "supplementary_tests/DynamicBehaviourCreator.h"
#include <test_supplementary.h>

#include "communication/AlicaRosCommunication.h"
#include <engine/BasicBehaviour.h>
#include <engine/modelmanagement/factories/BehaviourFactory.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{
TEST(ForceLoad, simple_load)
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
} // namespace
} // namespace alica
