#define BP_DEBUG

#include "engine/BehaviourPool.h"
#include "engine/AlicaEngine.h"
#include "engine/BasicBehaviour.h"
#include "engine/IBehaviourCreator.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/model/Behaviour.h"
#include "engine/model/BehaviourConfiguration.h"

namespace alica
{

/**
 * Basic Constructor.
 */
BehaviourPool::BehaviourPool(AlicaEngine* ae)
{
    this->ae = ae;
    this->behaviourCreator = nullptr;
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
    if (this->behaviourCreator != nullptr) {
        delete this->behaviourCreator;
    }

    this->behaviourCreator = bc;

    const PlanRepository::Accessor<Behaviour>& behaviours =
            ae->getPlanRepository()->getBehaviours();
    for (const Behaviour* beh : behaviours) {
        auto basicBeh = this->behaviourCreator->createBehaviour(beh->getId());
        if (basicBeh != nullptr) {
            // set stuff from behaviour configuration in basic behaviour object
            basicBeh->setBehaviour(beh);
            basicBeh->setDelayedStart(beh->getDeferring());
            basicBeh->setInterval(1000 / beh->getFrequency());
            basicBeh->setEngine(this->ae);
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
void BehaviourPool::stopAll() {
    const PlanRepository::Accessor<Behaviour>& behaviours = ae->getPlanRepository()->getBehaviours();
    for (const Behaviour* beh : behaviours) {
        auto bbPtr = _availableBehaviours.at(beh);
        if (bbPtr == nullptr) {
            std::cerr << "BP::stop(): Found Behaviour without an BasicBehaviour attached!" << std::endl;
            continue;
        }

        bbPtr->stop();
    }
}

/**
 * Enables the thread of the BasicBehaviour in the given RunningPlan.
 * @param rp A RunningPlan, which should represent a BehaviourConfiguration.
 */
void BehaviourPool::startBehaviour(std::shared_ptr<RunningPlan> rp) {
    if (const Behaviour* beh = dynamic_cast<const Behaviour*>(rp->getPlan())) {
        auto bb = _availableBehaviours.at(beh);
        if (bb != nullptr) {
            // set both directions rp <-> bb
            rp->setBasicBehaviour(bb);
            bb->setRunningPlan(rp);

            bb->start();
        }
    } else {
        std::cerr << "BP::startBehaviour(): Cannot start Behaviour of given RunningPlan! Plan Name: " << rp->getPlan()->getName()
                  << " Plan Id: " << rp->getPlan()->getId() << std::endl;
    }
}

/**
 * Disables the thread of the BasicBehaviour in the given RunningPlan.
 * @param rp A RunningPlan, which should represent a BehaviourConfiguration.
 */
void BehaviourPool::stopBehaviour(std::shared_ptr<RunningPlan> rp) {
    if (const Behaviour* beh = dynamic_cast<const Behaviour*>(rp->getPlan())) {
        auto bb = _availableBehaviours.at(beh);
        if (bb != nullptr) {
            bb->stop();
        }
    } else {
        std::cerr << "BP::stopBehaviour(): Cannot stop Behaviour of given RunningPlan! Plan Name: " << rp->getPlan()->getName()
                  << " Plan Id: " << rp->getPlan()->getId() << std::endl;
    }
}

const std::map<const Behaviour*, std::shared_ptr<BasicBehaviour>>& BehaviourPool::getAvailableBehaviours()
        const {
    return _availableBehaviours;
}

} /* namespace alica */
