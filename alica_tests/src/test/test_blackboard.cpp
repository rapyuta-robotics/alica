#include "test_alica.h"

namespace alica
{
namespace
{

class TestBlackBoard : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "TestParameterPassingMaster"; }
    bool stepEngine() const override { return false; }
};

TEST_F(TestBlackBoard, testRegisterValue)
{
    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    ae->start();
    ae->getAlicaClock().sleep(sleepTime * 1);
}
}
} // namespace alica

