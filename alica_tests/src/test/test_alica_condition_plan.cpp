#include "test_alica.h"
#include <alica_tests/Behaviour/Attack.h>
#include <alica_tests/Behaviour/ConstraintUsingBehaviour.h>
#include <alica_tests/CounterClass.h>

#include <alica/test/Util.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/Assignment.h>
#include <engine/BasicBehaviour.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/PlanBase.h>
#include <engine/PlanRepository.h>
#include <engine/TeamObserver.h>
#include <engine/constraintmodul/Query.h>
#include <engine/model/Behaviour.h>
#include <engine/model/Plan.h>
#include <engine/model/RuntimeCondition.h>
#include <engine/model/State.h>

#include <gtest/gtest.h>

#include <iostream>
#include <thread>

namespace alica
{
namespace
{

class AlicaConditionPlan : public AlicaTestFixtureWithSolvers
{
protected:
    const char* getMasterPlanName() const override { return "ConstraintTestPlan"; }
};
/**
 * Tests if Behaviour with Constraints are called
 */
TEST_F(AlicaConditionPlan, solverTest)
{
    ASSERT_NO_SIGNAL

    const alica::Behaviour* beh = ae->getPlanRepository().getBehaviours().find(1414068597716);
    ASSERT_NE(beh, nullptr);
    const alica::State* state = ae->getPlanRepository().getStates().find(1414068524246);
    ASSERT_NE(state, nullptr);

    ASSERT_EQ(beh->getVariables().size(), 2u);
    ASSERT_EQ(state->getParametrisation().size(), 2u);
    const alica::Variable* beh_y = nullptr;
    for (const alica::Variable* v : beh->getVariables()) {
        if (v->getName() == "Y") {
            beh_y = v;
            break;
        }
    }
    ASSERT_NE(beh_y, nullptr);

    ASSERT_EQ(beh_y->getId(), 1416488161203);
    bool found = false;
    for (const alica::VariableBinding* p : state->getParametrisation()) {
        ASSERT_EQ(p->getSubPlan(), beh);
        if (p->getSubVar() == beh_y) {
            found = true;
        }
    }
    ASSERT_TRUE(found) << "Sub variable not found in parametrisation";

    ae->start();
    ac->stepEngine();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));

    auto constraintUsingBehaviour = dynamic_cast<alica::ConstraintUsingBehaviour*>(alica::test::Util::getBasicBehaviour(ae, 1414068597716, 0));
    ASSERT_NE(constraintUsingBehaviour, nullptr);
    ASSERT_GT(constraintUsingBehaviour->getCallCounter(), 0);

    ASSERT_GT(alica::reasoner::ConstraintTestPlanDummySolver::getGetSolutionCallCounter(), 0);
    ASSERT_EQ(alica::ConstraintUsingBehaviour::result.size(), 1u) << "Wrong result size";
    int64_t res = LockedBlackboardRO(ae->getBlackboard()).get<int64_t>(std::to_string(alica::ConstraintUsingBehaviour::result[0]));
    EXPECT_EQ(1414068576620, res); // id of variable at highest level
}
} // namespace
} // namespace alica
