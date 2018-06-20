#include "ConstraintTestPlanDummySolver.h"
#include "Plans/GSolver/SolverTestBehaviour.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/BehaviourPool.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/model/Behaviour.h"
#include "engine/model/Plan.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/State.h"
#include <CGSolver.h>
#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/constraintmodul/Query.h>
#include <gtest/gtest.h>
#include <iostream>
#include <test_alica.h>
#include <thread>

class AlicaGSolverPlan : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "GSolverMaster"; }
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
 * Tests if Behaviour with Constraints are called
 */
TEST_F(AlicaGSolverPlan, solverTest)
{
    ASSERT_NO_SIGNAL

    std::cout << "Starting engine..." << std::endl;
    ae->start();

    step(ae);

    ASSERT_EQ(alica::SolverTestBehaviour::result.size(), 2) << "Wrong result size";
    EXPECT_GT(alica::SolverTestBehaviour::result[0], 4000);
    EXPECT_LT(alica::SolverTestBehaviour::result[0], 5000);
    EXPECT_GT(alica::SolverTestBehaviour::result[1], 7000);
    EXPECT_LT(alica::SolverTestBehaviour::result[1], 8000);
}
