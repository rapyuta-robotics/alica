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
TestContext::TestContext(const std::string& agentName, const std::string& configPath, const std::string& roleSetName, const std::string& masterPlanName,
        bool stepEngine, const AgentId agentID)
        : AlicaContext(AlicaContextParams(agentName, configPath, roleSetName, masterPlanName, stepEngine, agentID))
        , _initCalled(false)
{
}

int TestContext::init(AlicaCreators& creatorCtx)
{
    AlicaCreators creators(std::move(creatorCtx.conditionCreator), std::move(creatorCtx.utilityCreator), std::move(creatorCtx.constraintCreator),
            std::move(creatorCtx.behaviourCreator), std::move(creatorCtx.planCreator), std::move(creatorCtx.transitionConditionCreator));
    return init(std::move(creators));
}

int TestContext::init(AlicaCreators&& creatorCtx)
{
    _initCalled = true;
    if (_communicator) {
        _communicator->startCommunication();
    }

    if (AlicaContext::init(std::move(creatorCtx))) {
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
