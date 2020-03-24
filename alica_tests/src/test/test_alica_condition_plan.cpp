#include "CounterClass.h"
#include "Behaviour/ConstraintUsingBehaviour.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/BehaviourPool.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/model/Behaviour.h"
//#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/Plan.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/State.h"
#include <Behaviour/Attack.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/constraintmodul/Query.h>
#include <gtest/gtest.h>
#include <iostream>
#include <test_alica.h>
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

    const alica::PlanRepository& rep = ae->getPlanRepository();

    const alica::Behaviour* beh = rep.getBehaviours()[1414068597716];
    ASSERT_NE(beh, nullptr);
    const alica::State* state = rep.getStates()[1414068524246];
    ASSERT_NE(state, nullptr);

    ASSERT_EQ(beh->getVariables().size(), 2);
    ASSERT_EQ(state->getParametrisation().size(), 2);
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
    step(ae);

    alica::BasicBehaviour* basicBehaviour = ae->getPlanBase().getRootNode()->getChildren()[0]->getBasicBehaviour();
    alica::ConstraintUsingBehaviour* constraintUsingBehaviour = dynamic_cast<alica::ConstraintUsingBehaviour*>(basicBehaviour);
    ASSERT_NE(constraintUsingBehaviour, nullptr);
    ASSERT_GT(constraintUsingBehaviour->getCallCounter(), 0);

    ASSERT_GT(alica::reasoner::ConstraintTestPlanDummySolver::getGetSolutionCallCounter(), 0);
    ASSERT_EQ(alica::ConstraintUsingBehaviour::result.size(), 1) << "Wrong result size";
    const alica::ByteArray& ba = ae->getBlackBoard().getValue(alica::ConstraintUsingBehaviour::result[0]);
    std::string resultingString(reinterpret_cast<const char*>(ba.begin()), ba.size());
    EXPECT_EQ("1414068576620", resultingString); // id of variable at highest level
}
}
}