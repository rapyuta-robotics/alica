#include "engine/BehaviourPool.h"
#include "engine/AlicaEngine.h"
#include "engine/BasicBehaviour.h"
#include "engine/IBehaviourCreator.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/model/Behaviour.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/Configuration.h"

//#define ALICA_DEBUG_LEVEL_ALL
#include <alica_common_config/debug_output.h>

namespace alica
{

/**
 * Basic Constructor.
 */
BehaviourPool::BehaviourPool(AlicaEngine* ae)
        : _ae(ae)

{
}

/**
 * Basic Destructor.
 */
BehaviourPool::~BehaviourPool() = default;

/**
 * Creates instances of BasicBehaviours, needed according to the PlanRepository, with the help of the given
 * BehaviourCreator. If a BasicBehaviour cannot be instantiated, the Initialisation of the Pool is cancelled.
 * @param bc A BehaviourCreator.
 * @return True, if all necessary BasicBehaviours could be constructed. False, if the Initialisation was cancelled.
 */
bool BehaviourPool::init(IBehaviourCreator& bc)
{
    const PlanRepository::Accessor<ConfAbstractPlanWrapper>& wrappers = _ae->getPlanRepository().getConfAbstractPlanWrappers();
    for (const ConfAbstractPlanWrapper* wrapper : wrappers) {
        if (const auto* behaviour = dynamic_cast<const Behaviour*>(wrapper->getAbstractPlan())) {
            if (getBasicBehaviour(behaviour, wrapper->getConfiguration())) {
                // A BasicBehaviour representing this combination of Behaviour and Configuration was created already!
                continue;
            }

            auto basicBeh = bc.createBehaviour(behaviour->getId());
            if (!basicBeh) {
                // probably something with the (autogenerated) behaviour creator is wrong
                return false;
            }

            // set stuff from behaviour and configuration in basic behaviour object
            basicBeh->setConfiguration(wrapper->getConfiguration());
            basicBeh->setBehaviour(behaviour);
            basicBeh->setDelayedStart(behaviour->getDeferring());
            basicBeh->setInterval(1000 / (behaviour->getFrequency() < 1 ? 1 : behaviour->getFrequency()));
            basicBeh->setEngine(_ae);
            basicBeh->init();
            _availableBehaviours.insert(std::make_pair(wrapper, basicBeh));
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
    if (const auto* beh = dynamic_cast<const Behaviour*>(rp.getActivePlan())) {
        if (auto& bb = getBasicBehaviour(beh, rp.getConfiguration())) {
            // set both directions rp <-> bb
            rp.setBasicBehaviour(bb.get());
            bb->setRunningPlan(&rp);

//            bb->start();
            return;
        }
    }

    ALICA_ERROR_MSG("BP::startBehaviour(): Cannot start Behaviour of given RunningPlan! Plan Name: " << rp.getActivePlan()->getName()
                                                                                                     << " Plan Id: " << rp.getActivePlan()->getId());
}

/**
 * Disables the thread of the BasicBehaviour in the given RunningPlan.
 * @param rp A RunningPlan, which should represent a BehaviourConfiguration.
 */
void BehaviourPool::stopBehaviour(RunningPlan& rp)
{
    if (const auto* beh = dynamic_cast<const Behaviour*>(rp.getActivePlan())) {
        if (auto& bb = getBasicBehaviour(beh, rp.getConfiguration())) {
            bb->stop();
        }
    } else {
        ALICA_ERROR_MSG("BP::stopBehaviour(): Cannot stop Behaviour of given RunningPlan! Plan Name: " << rp.getActivePlan()->getName()
                                                                                                       << " Plan Id: " << rp.getActivePlan()->getId());
    }
}

bool BehaviourPool::isBehaviourRunningInContext(const RunningPlan& rp) const
{
    if (const auto* beh = dynamic_cast<const Behaviour*>(rp.getActivePlan())) {
        auto& bb = getBasicBehaviour(beh, rp.getConfiguration());
        return bb && bb->isRunningInContext(&rp);
    }
    return false;
}

const std::shared_ptr<BasicBehaviour> BehaviourPool::getBasicBehaviour(const Behaviour* behaviour, const Configuration* configuration) const
{
    for (const auto& poolEntry : _availableBehaviours) {
        if (poolEntry.first->getAbstractPlan() == behaviour && poolEntry.first->getConfiguration() == configuration) {
            return poolEntry.second;
        }
    }
    return nullptr;
}

} /* namespace alica */
