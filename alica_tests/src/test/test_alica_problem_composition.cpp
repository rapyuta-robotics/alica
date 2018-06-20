#include <BehaviourCreator.h>
#include <CGSolver.h>
#include <ConditionCreator.h>
#include <ConstraintCreator.h>
#include <ConstraintTestPlanDummySolver.h>
#include <FileSystem.h>
#include <Plans/ProblemModule/QueryBehaviour1.h>
#include <SystemConfig.h>
#include <UtilityFunctionCreator.h>
#include <chrono>
#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/PlanBase.h>
#include <engine/RunningPlan.h>
#include <engine/constraintmodul/Query.h>
#include <engine/model/Variable.h>
#include <gtest/gtest.h>
#include <gtest/internal/gtest-internal.h>
#include <iostream>
#include <memory>
#include <string>
#include <test_alica.h>
#include <thread>
#include <vector>

class AlicaProblemCompositionTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "ProblemBuildingMaster"; }
    void SetUp() override
    {
        AlicaTestFixture::SetUp();
        ae->addSolver(new alica::reasoner::ConstraintTestPlanDummySolver(ae));
        ae->addSolver(new alica::reasoner::CGSolver(ae));
    }
    void TearDown() override
    {
        alica::ISolverBase* s1 = ae->getSolver<alica::reasoner::ConstraintTestPlanDummySolver>();
        alica::ISolverBase* s2 = ae->getSolver<alica::reasoner::CGSolver>();
        AlicaTestFixture::TearDown();
        delete s1;
        delete s2;
    }
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

    const RunningPlan* deep = ae->getPlanBase()->getDeepestNode();

    ASSERT_FALSE(deep == nullptr);
    ASSERT_EQ(deep->getChildren().size(), 1);
    ASSERT_TRUE((*deep->getChildren().begin())->isBehaviour());

    auto queryBehaviour1 = dynamic_cast<QueryBehaviour1*>((*deep->getChildren().begin())->getBasicBehaviour());
    ASSERT_NE(queryBehaviour1, nullptr);
    VariableGrp allReps;
    queryBehaviour1->query->getUniqueVariableStore().getAllRep(allReps);

    for (const alica::Variable* rep : allReps) {
        EXPECT_TRUE(rep->getName() == "PBMX" || rep->getName() == "PBMY");
        // cout << "Test: '" << rep->getName() << "'" << endl;
    }
}
