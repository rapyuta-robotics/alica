#include "test_alica.h"

#include <alica/test/Util.h>
#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaSchedulingPlan : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SchedulingTestMasterPlan"; }
    bool stepEngine() const override { return false; }
};

TEST_F(AlicaSchedulingPlan, scheduling)
{
    std::cerr << "RUN SCHEDULING TEST" << std::endl;
    ae->start();
    ac->stepEngine();

    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    ae->getAlicaClock().sleep(sleepTime);

//    do {
//        ae->getAlicaClock().sleep(sleepTime);
//    } while (!alica::test::Util::isPlanActive(ae, 1613378406860));
//    do {
//    ae->getAlicaClock().sleep(sleepTime);
//    } while (!alica::test::Util::isPlanActive(ae, 1412252439925));
}

} //namespace
} //namespace alica




