#include "alica/test/TestContext.h"

#include "alica/test/TestBehaviourCreator.h"

#include <engine/BasicBehaviour.h>
#include <engine/IRoleAssignment.h>
#include <engine/model/Behaviour.h>
#include <engine/model/ConfAbstractPlanWrapper.h>
#include <engine/model/Configuration.h>

namespace alica::test
{

TestContext::TestContext(const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine, const essentials::IdentifierConstPtr agentID)
        : AlicaContext(roleSetName, masterPlanName, stepEngine, agentID)
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
    assert(_initCalled == false);
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
    // create and set a trigger, if none is passed
    std::shared_ptr<essentials::ITrigger> behaviourTrigger;
    if (!trigger) {
        behaviourTrigger = createStandardBehaviourTrigger(behaviourID, configurationID);
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
    std::shared_ptr<alica::BasicBehaviour> behaviour = nullptr;
    for (auto& behaviourEntry : _engine->getBehaviourPool().getAvailableBehaviours()) {
        if (behaviourEntry.first->getAbstractPlan()->getId() == behaviourID &&
                (configurationID == 0 ? behaviourEntry.first->getConfiguration() == nullptr
                                      : behaviourEntry.first->getConfiguration()->getId() == configurationID)) {
            behaviour = behaviourEntry.second;
        }
    }
    return behaviour;
}

void TestContext::handleAgentAnnouncement(alica::AgentAnnouncement agentAnnouncement)
{
    _engine->editTeamManager().handleAgentAnnouncement(agentAnnouncement);
}

void TestContext::setTeamTimeout(AlicaTime timeout)
{
    _engine->editTeamManager().setTeamTimeout(timeout);
}

void TestContext::tickTeamManager()
{
    _engine->editTeamManager().tick();
}

void TestContext::tickTeamObserver(alica::RunningPlan* root)
{
    _engine->editTeamObserver().tick(root);
}

void TestContext::tickRoleAssignment()
{
    _engine->editRoleAssignment().tick();
}

alica::PlanSelector* TestContext::getPlanSelector()
{
    return _engine->getPlanBase().getPlanSelector();
}

alica::RunningPlan* TestContext::makeRunningPlan(const alica::Plan* plan, const alica::Configuration* configuration)
{
    return _engine->editPlanBase().makeRunningPlan(plan, configuration);
}

VariableSyncModule& TestContext::editResultStore()
{
    return _engine->editResultStore();
}

//////////////////////////////////////////////////////
// Getter for tests ... everything returned is const//
//////////////////////////////////////////////////////

const RunningPlan* TestContext::getDeepestNode()
{
    return _engine->getPlanBase().getDeepestNode();
}

const RunningPlan* TestContext::getRootNode()
{
    return _engine->getPlanBase().getRootNode();
}

const BlackBoard& TestContext::getBlackBoard()
{
    return _engine->getBlackBoard();
}

// const BehaviourPool& TestContext::getBehaviourPool()
//{
//    return _engine->getBehaviourPool();
//}

int TestContext::getTeamSize()
{
    return _engine->getTeamManager().getTeamSize();
}

const DomainVariable* TestContext::getDomainVariable(essentials::IdentifierConstPtr agentID, std::string sort)
{
    return _engine->getTeamManager().getDomainVariable(agentID, sort);
}

const alica::Agent* TestContext::getLocalAgent()
{
    return _engine->getTeamManager().getLocalAgent();
}

const alica::Agent* TestContext::getAgentByID(essentials::IdentifierConstPtr agentID)
{
    return _engine->getTeamManager().getAgentByID(agentID);
}

const Plan* TestContext::getPlan(int64_t planID)
{
    return _engine->getPlanRepository().getPlans().find(planID);
}

const alica::PlanRepository::Accessor<Plan> TestContext::getPlans()
{
    return _engine->getPlanRepository().getPlans();
}

const Behaviour* TestContext::getBehaviour(int64_t behaviourID)
{
    return _engine->getPlanRepository().getBehaviours().find(behaviourID);
}

const alica::PlanRepository::Accessor<Behaviour> TestContext::getBehaviours()
{
    return _engine->getPlanRepository().getBehaviours();
}

const State* TestContext::getState(int64_t stateID)
{
    return _engine->getPlanRepository().getStates().find(stateID);
}

//////////////////////////////////////////////////////
// Private methods ...                              //
//////////////////////////////////////////////////////

std::shared_ptr<BehaviourTrigger> TestContext::createStandardBehaviourTrigger(int64_t behaviourID, int64_t configurationID)
{
    std::shared_ptr<alica::BasicBehaviour> behaviour = getBasicBehaviour(behaviourID, configurationID);
    if (behaviour) {
        std::shared_ptr<BehaviourTrigger> trigger = std::make_shared<BehaviourTrigger>();
        behaviour->setTrigger(trigger.get());
        return trigger;
    } else {
        return nullptr;
    }
}

} // namespace alica::test
