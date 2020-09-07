
#include "engine/DefaultUtilityFunction.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/model/Behaviour.h"
#include "engine/model/Plan.h"
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <gtest/gtest.h>
#include <test_alica.h>

namespace alica
{
namespace
{

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

    for (const alica::Behaviour* behaviour : ae->getPlanRepository().getBehaviours()) {
        ASSERT_NE(behaviour, nullptr);
        std::cout << "Behaviour: " << behaviour->getName() << std::endl;
    }
}
}
}