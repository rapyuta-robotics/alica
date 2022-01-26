#include "engine/PlanPool.h"
#include "engine/AlicaEngine.h"
#include "engine/BasicPlan.h"
#include "engine/IPlanCreator.h"
#include "engine/PlanRepository.h"
#include "engine/RunningPlan.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/Configuration.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"

//#define ALICA_DEBUG_LEVEL_ALL
#include <alica_common_config/debug_output.h>

namespace alica
{

/**
 * Basic Constructor.
 */
PlanPool::PlanPool(AlicaEngine* ae)
        : _ae(ae)

{
}

/**
 * Basic Destructor.
 */
PlanPool::~PlanPool() = default;

std::unique_ptr<BasicPlan> PlanPool::createBasicPlan(IPlanCreator& planCreator, const Plan* plan, const Configuration* configuration)
{
    auto basicPlan = planCreator.createPlan(plan->getId(), _ae->getWorldModel());
    if (!basicPlan) {
        // probably something with the (autogenerated) plan creator is wrong
        return basicPlan;
    }

    // set stuff from plan and configuration in basic plan object
    basicPlan->setConfiguration(configuration);
    basicPlan->setName(plan->getName());
    basicPlan->addKeyMappings(plan);
    if (plan->getFrequency() < 1) {
        basicPlan->setInterval(0);
    } else {
        basicPlan->setInterval(1000 / plan->getFrequency());
    }
    basicPlan->setInheritBlackboard(plan->getInheritBlackboard());
    basicPlan->setBlackboardBlueprint(plan->getBlackboardBlueprint());

    basicPlan->setEngine(_ae);
    if (plan->isMasterPlan()) {
        basicPlan->setAsMasterPlan();
    }

    return basicPlan;
}

/**
 * Creates instances of BasicPlan, needed according to the PlanRepository, with the help of the given
 * PlanCreator. If a BasicPlan cannot be instantiated, the Initialisation of the Pool is cancelled.
 * @param planCreator A PlanCreator.
 * @return True, if all necessary BasicPlan could be constructed. False, if the Initialisation was cancelled.
 */
bool PlanPool::init(IPlanCreator& planCreator)
{
    for (auto plan : _ae->getPlanRepository().getPlans()) {
        if (plan->isMasterPlan()) {
            _masterPlan = createBasicPlan(planCreator, plan, nullptr);
        }
    }

    const PlanRepository::Accessor<ConfAbstractPlanWrapper>& wrappers = _ae->getPlanRepository().getConfAbstractPlanWrappers();
    for (const ConfAbstractPlanWrapper* wrapper : wrappers) {
        if (const auto* plan = dynamic_cast<const Plan*>(wrapper->getAbstractPlan())) {
            if (getBasicPlan(plan, wrapper->getConfiguration())) {
                // A BasicPlan representing this combination of Plan and Configuration was created already!
                continue;
            }

            auto basicPlan = createBasicPlan(planCreator, plan, wrapper->getConfiguration());
            if (basicPlan) {
                _availablePlans.insert({{plan, wrapper->getConfiguration()}, std::move(basicPlan)});
            }
        } else if (const PlanType* pt = dynamic_cast<const PlanType*>(wrapper->getAbstractPlan())) {
            for (const auto* plan : pt->getPlans()) {
                if (getBasicPlan(plan, wrapper->getConfiguration())) {
                    // A BasicPlan representing this combination of Plan and Configuration was created already!
                    continue;
                }

                auto basicPlan = createBasicPlan(planCreator, plan, wrapper->getConfiguration());
                if (basicPlan) {
                    _availablePlans.insert({{plan, wrapper->getConfiguration()}, std::move(basicPlan)});
                }
            }
        }
    }
    return true;
}

/**
 * Enables the thread of the BasicPlan in the given RunningPlan.
 * @param rp A RunningPlan, which should represent a PlanConfiguration.
 */
void PlanPool::startPlan(RunningPlan& rp)
{
    if (const auto* plan = dynamic_cast<const Plan*>(rp.getActivePlan())) {
        if (auto basicPlan = getBasicPlan(plan, rp.getConfiguration())) {
            // set both directions rp <-> basicPlan
            rp.setBasicPlan(basicPlan);
            basicPlan->start(&rp);
            return;
        }
    }

    ALICA_ERROR_MSG("PP::startPlan(): Cannot start Plan of given RunningPlan! Plan Name: " << rp.getActivePlan()->getName()
                                                                                           << " Plan Id: " << rp.getActivePlan()->getId());
}

/**
 * Disables the thread of the BasicPlan in the given RunningPlan.
 * @param rp A RunningPlan, which should represent a PlanConfiguration.
 */
void PlanPool::stopPlan(RunningPlan& rp)
{
    if (const auto* plan = dynamic_cast<const Plan*>(rp.getActivePlan())) {
        if (auto basicPlan = getBasicPlan(plan, rp.getConfiguration())) {
            basicPlan->stop();
        }
    } else {
        ALICA_ERROR_MSG("PP::stopPlan(): Cannot stop Plan of given RunningPlan! Plan Name: " << rp.getActivePlan()->getName()
                                                                                             << " Plan Id: " << rp.getActivePlan()->getId());
    }
}

/**
 * Calls stop on all BasicPlans.
 */
void PlanPool::stopAll()
{
    if (_masterPlan) {
        _masterPlan->stop();
    }
    for (auto& plan_pair : _availablePlans) {
        plan_pair.second->stop();
    }
}

BasicPlan* PlanPool::getBasicPlan(const Plan* plan, const Configuration* configuration) const
{
    if (plan->isMasterPlan()) {
        return _masterPlan.get();
    }
    auto it = _availablePlans.find({plan, configuration});
    if (it != _availablePlans.end()) {
        return it->second.get();
    }
    return nullptr;
}

} /* namespace alica */
