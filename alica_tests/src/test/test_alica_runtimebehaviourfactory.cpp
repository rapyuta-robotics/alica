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

class AlicaRuntimeBehaviour : public AlicaTestMultiAgentFixture
{
protected:
    const int agentCount = 2;
    const char* getRoleSetName() const override { return "RolesetTA"; }
    const char* getMasterPlanName() const override { return "MultiAgentTestMaster"; }
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "hairy";
        } else {
            return "nase";
        }
    }
};

TEST_F(AlicaRuntimeBehaviour, scheduling)
{
    ros::NodeHandle nh;
    std::string path;
    nh.param<std::string>("/rootPath", path, ".");

    YAML::Node node;
    try {
        node = YAML::LoadFile(path+"/etc/plans/behaviours/ForceLoad.beh");
    } catch (YAML::BadFile& badFile) {
        AlicaEngine::abort("MM: Could not parse file: ", badFile.msg);
    }
    Behaviour* behaviourModel;
    behaviourModel = BehaviourFactory::create(nullptr, node);
    
    RuntimeBehaviourFactory rtbf(std::make_unique<alica::BehaviourCreator>(), nullptr, nullptr);
    rtbf.create(1234, behaviourModel);
    std::cerr<<"END**************************************"<<std::endl;
}
} // namespace
} // namespace alica