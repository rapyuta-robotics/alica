#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include <alica_tests/ConstraintTestPlanDummySolver.h>
#include <alica_tests/TestWorldModel.h>
#include "UtilityFunctionCreator.h"

#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaContext.h>
#include <engine/AlicaEngine.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <csetjmp>
#include <csignal>
#include <string>

#include <yaml-cpp/yaml.h>

#include "test_alica.h"
namespace alica {

namespace {

class AlicaTestYamlConfig : public AlicaTestFixture {
    const char *getMasterPlanName() const override
    {
        return "MasterPlan";
    };

    void TestBody() override
    {
        this->SetUp();
    };
};
}

void printNode(const YAML::Node& node, int depth)
{
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it){
        std::cerr << std::string(depth * 2, ' ');
        if (it->second.IsScalar()) {
            std::cerr << it->first << ": " << it->second << std::endl;
        } else {
            std::cerr << it->first << std::endl;
            printNode(it->second, depth + 1);
        }
    }
}

TEST_F(AlicaTestYamlConfig, CheckConfigInitialization)
{
    SetUp();
    const YAML::Node& node = ac->getConfig();
    printNode(node, 0);

    //Check if config has been initialized correctly
    EXPECT_EQ(false, node["Local"]["IsGoalie"].as<bool>());
    EXPECT_EQ(9, node["Local"]["ID"].as<int>());
    EXPECT_EQ(3000.0f, node["Local"]["MaxTranslation"].as<float>());

    EXPECT_EQ(30, node["Alica"]["EngineFrequency"].as<int>());
    EXPECT_EQ("plans", node["Alica"]["PlanDir"].as<std::string>());
    EXPECT_EQ(45, node["Alica"]["CycleDetection"]["HistorySize"].as<int>());
}

} //namespace alica