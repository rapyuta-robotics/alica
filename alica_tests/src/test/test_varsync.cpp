#include <engine/AlicaClock.h>
#include <engine/collections/Variant.h>
#include <engine/constraintmodul/VariableSyncModule.h>
#include <engine/model/Variable.h>

#include <alica_solver_interface/SolverVariable.h>

#include <test_alica.h>

#include <engine/PlanBase.h>

#include <essentials/SystemConfig.h>

#include <gtest/gtest.h>
#include <string.h>

using alica::Variable;
using alica::VariableSyncModule;
using alica::Variant;

namespace alica
{
namespace
{

class VariableSyncModuleTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "MasterPlan"; }
};

TEST_F(VariableSyncModuleTest, GetOwnSeed)
{
    ASSERT_NO_SIGNAL

    VariableSyncModule& vsm = ae->editResultStore();

    Variant v1(1.23);
    Variant v2(-10.0);
    vsm.postResult(1, v1);
    vsm.postResult(2, v2);

    std::vector<alica::Interval<double>> limits(2);

    limits[0] = alica::Interval<double>(-10, 10);
    limits[1] = alica::Interval<double>(-10, 10);

    std::vector<Variant> seeds;

    alica::SolverVariable sv1(1);
    alica::SolverVariable sv2(2);
    std::vector<alica::SolverVariable*> vs(2);
    vs[0] = &sv1;
    vs[1] = &sv2;

    int num = vsm.getSeeds(vs, limits, seeds);
    EXPECT_EQ(num, 1);
    EXPECT_EQ(seeds.size(), 2u);
    EXPECT_TRUE(seeds[0].isDouble());
    EXPECT_TRUE(seeds[1].isDouble());

    EXPECT_EQ(1.23, seeds[0].getDouble());
    EXPECT_EQ(-10.0, seeds[1].getDouble());
}
}
}