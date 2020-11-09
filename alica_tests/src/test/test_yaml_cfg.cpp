#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "ConstraintTestPlanDummySolver.h"
#include "TestWorldModel.h"
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

// Declare a test
TEST_F(AlicaTestYamlConfig, InitAlicaContextWithConfig)
{
    SetUp();
    const YAML::Node& node = ac->getConfig();
    printNode(node, 0);
}

} //namespace alica