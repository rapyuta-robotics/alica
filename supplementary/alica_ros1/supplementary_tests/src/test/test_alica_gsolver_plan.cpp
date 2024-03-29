#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/Assignment.h>
#include <engine/BasicBehaviour.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/constraintmodul/Query.h>
#include <engine/model/Behaviour.h>
#include <engine/model/Plan.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/model/State.h>
#include <libalica-supplementary-tests/GSolver/SolverTestBehaviour.h>
#include <test_supplementary.h>

#include <gtest/gtest.h>

#include <iostream>
#include <thread>

namespace supplementary
{
namespace
{

class AlicaGSolverPlan : public AlicaTestFixtureWithSolvers
{
protected:
    const char* getMasterPlanName() const override { return "GSolverMaster"; }
};

/**
 * Tests if Behaviour with Constraints are called
 */
TEST_F(AlicaGSolverPlan, solverTest)
{
    ASSERT_NO_SIGNAL

    std::cout << "Starting engine..." << std::endl;
    ae->start();

    ac->stepEngine();

    ASSERT_EQ(alica::SolverTestBehaviour::result.size(), 2u) << "Wrong result size";
    EXPECT_GT(alica::SolverTestBehaviour::result[0], 4000);
    EXPECT_LT(alica::SolverTestBehaviour::result[0], 5000);
    EXPECT_GT(alica::SolverTestBehaviour::result[1], 7000);
    EXPECT_LT(alica::SolverTestBehaviour::result[1], 8000);
}
} // namespace
} // namespace supplementary
