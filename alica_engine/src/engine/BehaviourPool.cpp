#define BP_DEBUG

#include "engine/BehaviourPool.h"
#include "engine/AlicaEngine.h"
#include "engine/BasicBehaviour.h"
#include "engine/IBehaviourCreator.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/model/Behaviour.h"
#include "engine/model/BehaviourConfiguration.h"

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

    const PlanRepository::Accessor<BehaviourConfiguration>& behaviourConfs = _ae->getPlanRepository()->getBehaviourConfigurations();
    for (const BehaviourConfiguration* beh : behaviourConfs) {
        auto basicBeh = _behaviourCreator->createBehaviour(beh->getId());
        if (basicBeh != nullptr) {
            // set stuff from behaviour configuration in basic behaviour object
            basicBeh->setConfiguration(beh);
            basicBeh->setDelayedStart(beh->getDeferring());
            basicBeh->setInterval(1000 / beh->getFrequency());
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
    const PlanRepository::Accessor<BehaviourConfiguration>& behaviourConfs = _ae->getPlanRepository()->getBehaviourConfigurations();
    for (const BehaviourConfiguration* beh : behaviourConfs) {
        auto bbPtr = _availableBehaviours.at(beh);
        if (bbPtr == nullptr) {
            ALICA_ERROR_MSG("BP::stop(): Found Behaviour without an BasicBehaviour attached!");
            continue;
        }

        bbPtr->stop();
    }
}

/**
 * Enables the thread of the BasicBehaviour in the given RunningPlan.
 * @param rp A RunningPlan, which should represent a BehaviourConfiguration.
 */
void BehaviourPool::startBehaviour(RunningPlan& rp)
{
    if (const BehaviourConfiguration* bc = dynamic_cast<const BehaviourConfiguration*>(rp.getActivePlan())) {
        const std::shared_ptr<BasicBehaviour>& bb = _availableBehaviours.at(bc);
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
    if (const BehaviourConfiguration* bc = dynamic_cast<const BehaviourConfiguration*>(rp.getActivePlan())) {
        auto bb = _availableBehaviours.at(bc);
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
    if (const BehaviourConfiguration* bc = dynamic_cast<const BehaviourConfiguration*>(rp.getActivePlan())) {
        const std::shared_ptr<BasicBehaviour>& bb = _availableBehaviours.at(bc);
        if (bb != nullptr) {
            if (bb->getPlanContext().mapsTo(&rp)) {
                return !bb->isProperlyStopped();
            }
        }
    }
    return false;
}

} /* namespace alica */
