#define BP_DEBUG

#include "engine/BehaviourPool.h"
#include "engine/AlicaEngine.h"
#include "engine/BasicBehaviour.h"
#include "engine/IBehaviourCreator.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/model/Behaviour.h"

#include <alica_common_config/debug_output.h>

namespace alica
{

/**
 * Basic Constructor.
 */
BehaviourPool::BehaviourPool(AlicaEngine* ae)
        : _ae(ae)
        , _behaviourCreator(nullptr)

{
    std::cout << "Construct behaviour pool" << std::endl;
}

/**
 * Basic Destructor.
 */
BehaviourPool::~BehaviourPool() {}

/**
 * Creates instances of BasicBehaviours, needed according to the PlanRepository, with the help of the given
 * BehaviourCreator. If a BasicBehaviour cannot be instantiated, the Initialisation of the Pool is cancelled.
 * @param bc A BehaviourCreator.
 * @return True, if all necessary BasicBehaviours could be constructed. False, if the Initialisation was cancelled.
 */
bool BehaviourPool::init(IBehaviourCreator* bc)
{
    if (_behaviourCreator != nullptr) {
        delete _behaviourCreator;
    }

    _behaviourCreator = bc;

    const PlanRepository::Accessor<Behaviour>& behaviours = _ae->getPlanRepository()->getBehaviours();
    for (const Behaviour* beh : behaviours) {
        auto basicBeh = _behaviourCreator->createBehaviour(beh->getId());
        if (basicBeh != nullptr) {
            // set stuff from behaviour configuration in basic behaviour object
            basicBeh->setBehaviour(beh);
            basicBeh->setDelayedStart(beh->getDeferring());
            std::cout << "BehaviourPool::init: " << beh->getName() << " freq: " << beh->getFrequency() << std::endl;
            basicBeh->setInterval(1000 / (beh->getFrequency() < 1 ? 1 : beh->getFrequency()));
            basicBeh->setEngine(_ae);
            basicBeh->init();
            _availableBehaviours.insert(make_pair(beh, basicBeh));
        } else {
            return false;
        }
    }
    return true;
}

/**
 * Calls stop on all BasicBehaviours.
 */
void BehaviourPool::stopAll()
{
    for (auto& beh_pair : _availableBehaviours) {
        beh_pair.second->stop();
    }
}

/**
 * Calls terminate on all BasicBehaviours.
 */
void BehaviourPool::terminateAll()
{
    for (auto& beh_pair : _availableBehaviours) {
        beh_pair.second->terminate();
    }
}

/**
 * Enables the thread of the BasicBehaviour in the given RunningPlan.
 * @param rp A RunningPlan, which should represent a BehaviourConfiguration.
 */
void BehaviourPool::startBehaviour(RunningPlan& rp)
{
    if (const Behaviour* beh = dynamic_cast<const Behaviour*>(rp.getActivePlan())) {
        const std::shared_ptr<BasicBehaviour>& bb = _availableBehaviours.at(beh);
        if (bb != nullptr) {
            // set both directions rp <-> bb
            rp.setBasicBehaviour(bb.get());
            bb->setRunningPlan(&rp);

            bb->start();
        }
    } else {
        ALICA_ERROR_MSG("BP::startBehaviour(): Cannot start Behaviour of given RunningPlan! Plan Name: " << rp.getActivePlan()->getName()
                                                                                                         << " Plan Id: " << rp.getActivePlan()->getId());
    }
}

/**
 * Disables the thread of the BasicBehaviour in the given RunningPlan.
 * @param rp A RunningPlan, which should represent a BehaviourConfiguration.
 */
void BehaviourPool::stopBehaviour(RunningPlan& rp)
{
    if (const Behaviour* beh = dynamic_cast<const Behaviour*>(rp.getActivePlan())) {
        auto bb = _availableBehaviours.at(beh);
        if (bb != nullptr) {
            bb->stop();
        }
    } else {
        ALICA_ERROR_MSG("BP::stopBehaviour(): Cannot stop Behaviour of given RunningPlan! Plan Name: " << rp.getActivePlan()->getName()
                                                                                                       << " Plan Id: " << rp.getActivePlan()->getId());
    }
}

bool BehaviourPool::isBehaviourRunningInContext(const RunningPlan& rp) const
{
    if (const Behaviour* beh = dynamic_cast<const Behaviour*>(rp.getActivePlan())) {
        const std::shared_ptr<BasicBehaviour>& bb = _availableBehaviours.at(beh);
        if (bb != nullptr) {
            return bb->isRunningInContext(&rp);
        }
    }
    return false;
}

} /* namespace alica */
