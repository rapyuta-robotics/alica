#include <iostream>
#include <typeinfo>
#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaClock.h>
#include <communication/AlicaRosCommunication.h>
#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/model/Plan.h"
#include "engine/DefaultUtilityFunction.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include <csignal>


class PlanBaseTest : public ::testing::Test {
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae;
    alica::BehaviourCreator* bc;
    alica::ConditionCreator* cc;
    alica::UtilityFunctionCreator* uc;
    alica::ConstraintCreator* crc;

    static void signal_handler(int signal) { EXPECT_FALSE(signal); }
    
    virtual void SetUp() {
        std::signal(SIGSEGV, signal_handler);

        // determine the path to the test config
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");

        // bring up the SystemConfig with the corresponding path
        sc = supplementary::SystemConfig::getInstance();
        sc->setHostname("nase");
        sc->setRootPath(path);
        sc->setConfigPath(path + "/etc");
        sc->setHostname("nase");

        // setup the engine
        ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "Roleset",
                "MasterPlan", ".", false);
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();
        ae->setAlicaClock(new alica::AlicaClock());
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
        ae->init(bc, cc, uc, crc);
    }

    virtual void TearDown() {
        ae->shutdown();
        sc->shutdown();
        delete ae->getCommunicator();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }
};
// Declare a test
TEST_F(PlanBaseTest, planBaseTest) {
    // TODO test something
    ae->start();
    sleep(1);
}
