#include <gtest/gtest.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaClock.h>
#include "engine/IAlicaCommunication.h"
#include "engine/model/Behaviour.h"
#include "engine/PlanRepository.h"
#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>
#include "engine/DefaultUtilityFunction.h"
#include "engine/model/Plan.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"

class AlicaEngineTestBehPool : public ::testing::Test {
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae;
    alica::BehaviourCreator* bc;
    alica::ConditionCreator* cc;
    alica::UtilityFunctionCreator* uc;
    alica::ConstraintCreator* crc;

    virtual void SetUp() {
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
        ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
    }

    virtual void TearDown() {
        ae->shutdown();
        sc->shutdown();
        delete ae->getCommunicator();
        delete ae->getIAlicaClock();
        delete cc;
        delete bc;
        delete uc;
        delete crc;
    }
};
/**
 * Tests the initialisation of the behaviourPool
 */
TEST_F(AlicaEngineTestBehPool, behaviourPoolInit) {
    EXPECT_TRUE(ae->init(bc, cc, uc, crc)) << "Unable to initialise the Alica Engine!";
    alica::BehaviourPool* bp = ae->getBehaviourPool();
    for (const Behaviour* behaviour : ae->getPlanRepository()->getBehaviours()) {
        ASSERT_NE(behaviour, nullptr);
        cout << "Behaviour: " << behaviour->getName() << endl;
    }
}
