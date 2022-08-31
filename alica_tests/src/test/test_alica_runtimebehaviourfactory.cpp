#include "test_alica.h"

#include <alica/test/Util.h>
#include <alica_tests/BehaviourCreator.h>
#include <alica_tests/TestWorldModel.h>

#include <engine/modelmanagement/factories/BehaviourFactory.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{
TEST(ForceLoad, simple_load)
{
    std::cerr << "BEGIN**************************************" << std::endl;

    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    YAML::Node node;
    try {
        node = YAML::LoadFile(path + "/etc/plans/behaviours/Acme.beh");
    } catch (YAML::BadFile& badFile) {
        AlicaEngine::abort("MM: Could not parse behaviour file: ", badFile.msg);
    }

    YAML::Node alicaConfig;
    try {
        alicaConfig = YAML::LoadFile(path + "/etc/hairy/Alica.yaml");
    } catch (YAML::BadFile& badFile) {
        AlicaEngine::abort("MM: Could not parse Alica.yaml file: ", badFile.msg);
    }

    Behaviour* behaviourModel;
    behaviourModel = BehaviourFactory::create(nullptr, node);

    ConfigChangeListener configChangeListener(alicaConfig);
    RuntimeBehaviourFactory rtbf(configChangeListener, std::make_unique<alica::BehaviourCreator>(), nullptr, nullptr);
    auto behaviour=rtbf.create(1234, behaviourModel);
    ASSERT_EQ("AcmeBehaviour",behaviour->getName());

    behaviour.release();
    std::cerr << "END**************************************" << std::endl;
}
} // namespace
} // namespace alica