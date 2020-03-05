#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "ConstraintCreator.h"
#include "UtilityFunctionCreator.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/PlanRepository.h"
#include "engine/model/Plan.h"
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <test_alica.h>

namespace alica
{
namespace
{

class AlicaEngineTestInit : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "MasterPlan"; }
    bool stepEngine() const override { return false; }
};

/**
 * Initialises an instance of the AlicaEngine and shuts it down again. This test is nice for basic memory leak testing.
 */
TEST_F(AlicaEngineTestInit, initAndShutdown)
{
    ASSERT_NO_SIGNAL
    EXPECT_NE(ae, nullptr);
}
}
}