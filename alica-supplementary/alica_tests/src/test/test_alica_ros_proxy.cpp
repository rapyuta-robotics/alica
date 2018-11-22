#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/model/Plan.h"
#include <communication/AlicaRosCommunication.h>
#include <gtest/gtest.h>
#include <iostream>
#include <test_alica.h>
#include <typeinfo>

class PlanBaseTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "MasterPlan"; }
    bool stepEngine() const override { return false; }
};

// Declare a test
TEST_F(PlanBaseTest, planBaseTest)
{
    // TODO test something
    ae->start();
    sleep(1);
}
