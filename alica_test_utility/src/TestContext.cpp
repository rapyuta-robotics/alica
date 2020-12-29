#include "alica/test/TestContext.h"

#include "alica/test/TestBehaviourCreator.h"
#include "alica/test/Util.h"

#include <engine/BasicBehaviour.h>
#include <engine/IRoleAssignment.h>
#include <engine/model/Behaviour.h>
#include <engine/model/ConfAbstractPlanWrapper.h>
#include <engine/model/Configuration.h>

namespace alica::test
{

TestContext::TestContext(const std::string& agentName, const std::string& configPath,
                         const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine,
                         const essentials::Identifier& agentID)
        : AlicaContext(AlicaContextParams(agentName, configPath, roleSetName, masterPlanName, stepEngine, agentID))
        , _initCalled(false)
{
}

int TestContext::init(AlicaCreators& creatorCtx)
{
    _initCalled = true;
    if (_communicator) {
        _communicator->startCommunication();
    }

    if (_engine->init(creatorCtx)) {
        return 0;
    }
    return -1;
}

void TestContext::startEngine()
{
    _engine->start();
}

bool TestContext::makeBehaviourEventDriven(int64_t behaviourID)
{
    assert(_initCalled == false &&
            "[TestContext] The method makeBehaviourEventDriven(behaviourID) must be called, before the TestContext is initialised via its init-Method!");
    const Behaviour* constBehaviour = _engine->getPlanRepository().getBehaviours().find(behaviourID);
    if (constBehaviour == nullptr) {
        return false;
    }
    const_cast<Behaviour*>(constBehaviour)->setEventDriven(true);
    return true;
}

std::shared_ptr<essentials::ITrigger> TestContext::addBehaviourTrigger(
        int64_t behaviourID, int64_t configurationID, std::shared_ptr<essentials::ITrigger> trigger)
{
    // create a trigger, if none is passed
    std::shared_ptr<essentials::ITrigger> behaviourTrigger;
    if (!trigger) {
        behaviourTrigger = std::make_shared<BehaviourTrigger>();
    } else {
        behaviourTrigger = trigger;
    }

    // register the trigger in the TestContext trigger-map
    auto behaviourConfKey = std::make_pair(behaviourID, configurationID);
    auto behaviourTriggerEntry = _behaviourTriggers.find(behaviourConfKey);
    if (behaviourTriggerEntry == _behaviourTriggers.end()) {
        _behaviourTriggers[behaviourConfKey] = behaviourTrigger;
    } else {
        std::cerr << "[TestContext] Trigger for behaviourID " << behaviourID << " and configurationID " << configurationID << " already exists!" << std::endl;
    }

    // return the registered trigger
    return behaviourTrigger;
}

std::shared_ptr<BasicBehaviour> TestContext::getBasicBehaviour(int64_t behaviourID, int64_t configurationID)
{
    assert(_initCalled == true);
    return Util::getBasicBehaviour(_engine.get(), behaviourID, configurationID);
}

int TestContext::getTeamSize()
{
    return Util::getTeamSize(_engine.get());
}

bool TestContext::isStateActive(int64_t id) const
{
    return Util::isStateActive(_engine.get(), id);
}

bool TestContext::isPlanActive(int64_t id) const
{
    return Util::isPlanActive(_engine.get(), id);
}

} // namespace alica::test
