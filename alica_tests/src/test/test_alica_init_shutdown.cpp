#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/AlicaClock.h>
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include "engine/PlanRepository.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/model/Plan.h"
#include <communication/AlicaDummyCommunication.h>
#include <ros/ros.h>
#include <csignal>


class AlicaEngineTestInit : public ::testing::Test {
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
        ae->setCommunicator(new alica_dummy_proxy::AlicaDummyCommunication(ae));
    }

    virtual void TearDown() {
        ae->shutdown();  
        delete ae->getCommunicator();
        delete crc;
        delete uc;
        delete cc;
        delete bc;
        delete ae;
        sc->shutdown();
    }
};

/**
 * Initialises an instance of the AlicaEngine and shuts it down again. This test is nice for basic memory leak testing.
 */
TEST_F(AlicaEngineTestInit, initAndShutdown) {
    EXPECT_TRUE(ae->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";
}
