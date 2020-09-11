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

void TestContext::setTeamTimeout(AlicaTime timeout)
{
    _engine->editTeamManager().setTeamTimeout(timeout);
}

VariableSyncModule& TestContext::editResultStore()
{
    return _engine->editResultStore();
}

//////////////////////////////////////////////////////
// Getter for tests ... everything returned is const//
//////////////////////////////////////////////////////

const BlackBoard& TestContext::getBlackBoard()
{
    return _engine->getBlackBoard();
}

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

bool TestContext::isPlanActive(int64_t id) const
{
    return isPlanActiveHelper(_engine->getPlanBase().getRootNode(), id);
}

//////////////////////////////////////////////////////
// Private methods ...                              //
//////////////////////////////////////////////////////

bool TestContext::isPlanActiveHelper(const RunningPlan* rp, int64_t id) const
{
    if (!rp) {
        return false;
    }

    const AbstractPlan* abstractPlan = rp->getActivePlan();
    if (abstractPlan && abstractPlan->getId() == id) {
        return true;
    }
    const std::vector<RunningPlan*>& children = rp->getChildren();
    for (int i = 0; i < static_cast<int>(children.size()); ++i) {
        if (isPlanActiveHelper(children[i], id)) {
            return true;
        }
    }
    return false;
}
} // namespace alica::test
