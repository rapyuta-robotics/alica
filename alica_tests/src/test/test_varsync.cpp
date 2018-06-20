#include <engine/AlicaClock.h>
#include <engine/collections/Variant.h>
#include <engine/constraintmodul/VariableSyncModule.h>
#include <engine/model/Variable.h>

#include <alica_solver_interface/SolverVariable.h>

#include <gtest/gtest.h>
#include <test_alica.h>

#include <BehaviourCreator.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaEngine.h>
#include <engine/PlanBase.h>
#include <engine/RunningPlan.h>

#include <SystemConfig.h>
#include <UtilityFunctionCreator.h>

#include <string.h>

using alica::Variable;
using alica::VariableSyncModule;
using alica::Variant;

class VariableSyncModuleTest : public ::testing::Test
{
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae;
    alica::BehaviourCreator* bc;
    alica::ConditionCreator* cc;
    alica::UtilityFunctionCreator* uc;
    alica::ConstraintCreator* crc;

    virtual void SetUp()
    {
        ros::NodeHandle nh;
        std::string path;
        nh.param<std::string>("/rootPath", path, ".");

        // bring up the SystemConfig with the corresponding path
        sc = supplementary::SystemConfig::getInstance();
        sc->setRootPath(path);
        sc->setConfigPath(path + "/etc");

        // setup the engine
        bc = new alica::BehaviourCreator();
        cc = new alica::ConditionCreator();
        uc = new alica::UtilityFunctionCreator();
        crc = new alica::ConstraintCreator();

        sc->setHostname("nase");
        ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "Roleset", "ProblemBuildingMaster", true);
        ae->setAlicaClock(new alica::AlicaClock());
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
        ae->init(bc, cc, uc, crc);
    }

    virtual void TearDown()
    {
        ae->shutdown();
        delete ae->getCommunicator();
        delete ae->getAlicaClock();

        delete ae;

        delete crc;
        delete uc;
        delete cc;
        delete bc;

        sc->shutdown();
    }
};

TEST_F(VariableSyncModuleTest, GetOwnSeed)
{
    ASSERT_NO_SIGNAL

    VariableSyncModule* vsm = ae->getResultStore();

    Variant v1(1.23);
    Variant v2(-10.0);
    vsm->postResult(1, v1);
    vsm->postResult(2, v2);

    std::vector<Interval<double>> limits(2);

    limits[0] = Interval<double>(-10, 10);
    limits[1] = Interval<double>(-10, 10);

    std::vector<Variant> seeds;

    SolverVariable sv1(1);
    SolverVariable sv2(2);
    std::vector<SolverVariable*> vs(2);
    vs[0] = &sv1;
    vs[1] = &sv2;

    int num = vsm->getSeeds(vs, limits, seeds);
    EXPECT_EQ(num, 1);
    EXPECT_EQ(seeds.size(), 2);
    EXPECT_TRUE(seeds[0].isDouble());
    EXPECT_TRUE(seeds[1].isDouble());

    EXPECT_EQ(1.23, seeds[0].getDouble());
    EXPECT_EQ(-10.0, seeds[1].getDouble());
}
