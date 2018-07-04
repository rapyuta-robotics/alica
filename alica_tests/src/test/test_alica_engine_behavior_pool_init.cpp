
#include "engine/DefaultUtilityFunction.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/model/Behaviour.h"
#include "engine/model/Plan.h"
#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <gtest/gtest.h>
#include <test_alica.h>

class AlicaEngineTestBehPool : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "MasterPlan"; }
};

/**
 * Tests the initialisation of the behaviourPool
 */
TEST_F(AlicaEngineTestBehPool, behaviourPoolInit)
{
    ASSERT_NO_SIGNAL

    for (const Behaviour* behaviour : ae->getPlanRepository()->getBehaviours()) {
        ASSERT_NE(behaviour, nullptr);
        cout << "Behaviour: " << behaviour->getName() << endl;
    }
}
