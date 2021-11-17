#include <communication/AlicaRosCommunication.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanRepository.h>
#include <engine/model/Plan.h>
#include <test_supplementary.h>

#include <gtest/gtest.h>

#include <iostream>
#include <typeinfo>

namespace supplementary
{
namespace
{

class PlanBaseTest : public AlicaTestFixture
{
protected:
    const char* getMasterPlanName() const override { return "GSolverMaster"; }
};

// Declare a test
TEST_F(PlanBaseTest, planBaseTest)
{
    // TODO test something
    ae->start();
    sleep(1);
}
} // namespace
} // namespace supplementary