#include <Plans/ProblemModule/QueryBehaviour1.h>
#include <engine/PlanBase.h>
#include <engine/model/Variable.h>
#include <test_supplementary.h>
#include <gtest/gtest.h>
#include <gtest/internal/gtest-internal.h>

#include <string>
#include <vector>
#include <chrono>

namespace supplementary
{
namespace
{

class AlicaProblemCompositionTest : public AlicaTestFixtureWithSolvers
{
protected:
    const char* getMasterPlanName() const override { return "ProblemBuildingMaster"; }
};

/**
 * Tests if static variables and binded correctly.
 */
TEST_F(AlicaProblemCompositionTest, SimpleStaticComposition)
{
    ASSERT_NO_SIGNAL

    ae->start();

    for (int i = 0; i < 6; ++i) {
        step(ae);
    }

    const alica::RunningPlan* deep = ae->getPlanBase().getDeepestNode();

    ASSERT_FALSE(deep == nullptr);
    ASSERT_EQ(deep->getChildren().size(), 1);
    ASSERT_TRUE((*deep->getChildren().begin())->isBehaviour());

    alica::QueryBehaviour1* queryBehaviour1 = dynamic_cast<alica::QueryBehaviour1*>((*deep->getChildren().begin())->getBasicBehaviour());
    ASSERT_NE(queryBehaviour1, nullptr);
    queryBehaviour1->stopQueries();
    alica::VariableGrp allReps;
    queryBehaviour1->query->getUniqueVariableStore().getAllRep(allReps);

    for (const alica::Variable* rep : allReps) {
        EXPECT_TRUE(rep->getName() == "PBMX" || rep->getName() == "PBMY");
        // cout << "Test: '" << rep->getName() << "'" << endl;
    }
}
}
}