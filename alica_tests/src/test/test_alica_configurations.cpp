#include "Behaviour/Attack.h"
#include "Behaviour/MidFieldStandard.h"
#include "CounterClass.h"
#include "engine/Assignment.h"
#include "engine/BasicBehaviour.h"
#include "engine/BehaviourPool.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanBase.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/model/Behaviour.h"
#include "engine/model/Plan.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/State.h"
#include "engine/model/ConfAbstractPlanWrapper.h"

#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>

#include <gtest/gtest.h>
#include <test_alica.h>

namespace alica
{
namespace
{

class AlicaConfigurationPlan : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "ConfigurationTestPlan"; }
    bool stepEngine() const override { return false; }
};

/**
 * Tests whether it is possible to read configured behaviours in a primitive plan.
 */
TEST_F(AlicaConfigurationPlan, runBehaviourConfigurationTest)
{
    ASSERT_NO_SIGNAL
    std::cout << "AlicaConfigurationPlan: engine start will be called now!" << std::endl;
    ae->start();
    std::cout << "AlicaConfigurationPlan: engine start finished!" << std::endl;
    alica::AlicaTime sleepTime = alica::AlicaTime::seconds(1);
    do {
        ae->getAlicaClock().sleep(sleepTime);
    } while (ae->getPlanBase().getRootNode() == nullptr);
    const std::vector<RunningPlan*> children = ae->getPlanBase().getRootNode()->getChildren();
    std::cout << "Root Name: " << ae->getPlanBase().getRootNode()->getActivePlan()->getName() << std::endl;
    for (auto& wrapper : ((Plan*)ae->getPlanBase().getRootNode()->getActivePlan())->getStates().at(0)->getConfAbstractPlanWrappers()) {
        std::cout << "Wrapper Plan Name: " << wrapper->getAbstractPlan()->getName() << std::endl;
    }
    for (const RunningPlan* rpChild : children) {
        std::cout << "AbstractPlan Name: " << rpChild->getActivePlan()->getName() << std::endl;
    }
}
} // namespace
} // namespace alica