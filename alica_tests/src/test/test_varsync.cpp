#include <test_alica.h>
#include <gtest/gtest.h>
#include <engine/AlicaClock.h>
#include <engine/collections/Variant.h>
#include <engine/constraintmodul/VariableSyncModule.h>
#include <engine/model/Variable.h>

#include <BehaviourCreator.h>
#include <communication/AlicaRosCommunication.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <engine/AlicaEngine.h>
#include <engine/PlanBase.h>
#include <engine/RunningPlan.h>

#include <SystemConfig.h>
#include <UtilityFunctionCreator.h>

#include <string.h>

using alica::Variable;
using alica::VariableSyncModule;
using alica::Variant;

class VariableSyncModuleTest : public ::testing::Test {
protected:
    supplementary::SystemConfig* sc;
    alica::AlicaEngine* ae;
    alica::BehaviourCreator* bc;
    alica::ConditionCreator* cc;
    alica::UtilityFunctionCreator* uc;
    alica::ConstraintCreator* crc;

    virtual void SetUp() {
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
        ae = new alica::AlicaEngine(new supplementary::AgentIDManager(new supplementary::AgentIDFactory()), "Roleset",
                "ProblemBuildingMaster", ".", true);
        ae->setAlicaClock(new alica::AlicaClock());
        ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
        ae->init(bc, cc, uc, crc);
    }

    virtual void TearDown() {
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

TEST_F(VariableSyncModuleTest, GetOwnSeed) {
    ASSERT_NO_SIGNAL

    VariableSyncModule* vsm = ae->getResultStore();

    //    virtual void postResult(int64_t vid, Variant result) override;
    // virtual int getSeeds(const VariableGrp& query, const std::vector<double>& limits, std::vector<Variant>& o_seeds)
    // const override;
    Variant v1(1.23);
    Variant v2(-10.0);
    vsm->postResult(1, v1);
    vsm->postResult(2, v2);

    Variable var1(1, "Var1", "");
    Variable var2(2, "Var2", "");
    VariableGrp vs(2);
    vs[0] = &var1;
    vs[1] = &var2;

    std::vector<double> limits(4);

    limits[0] = -10;
    limits[1] = 10;
    limits[2] = -10;
    limits[3] = 10;

    std::vector<Variant> seeds;

    int num = vsm->getSeeds(vs, limits, seeds);
    EXPECT_EQ(num, 1);
    EXPECT_EQ(seeds.size(), 2);
    EXPECT_TRUE(seeds[0].isDouble());
    EXPECT_TRUE(seeds[1].isDouble());

    EXPECT_EQ(1.23, seeds[0].getDouble());
    EXPECT_EQ(-10.0, seeds[1].getDouble());
}
