#include "engine/collections/SuccessMarks.h"

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"

namespace alica
{

/**
 * Default Constructor
 */
SuccessMarks::SuccessMarks() {}

SuccessMarks::~SuccessMarks() {}

/**
 * Update with an IdGrp of EntryPoint ids, as received by a message
 */
void SuccessMarks::update(const AlicaEngine* ae, const std::vector<std::size_t>& succeededContexts)
{
    clear();
    _successMarks.insert(succeededContexts.begin(), succeededContexts.end());
}

/**
 * Drop every mark not occurring in plans passed as argument.
 */
void SuccessMarks::limitToPlans(const AbstractPlanGrp& active)
{
    AbstractPlanGrp tr;

    for (const auto& successMarkEntry : _successMarks) {
        if (std::find(active.begin(), active.end(), successMarkEntry.first) == active.end()) {
            tr.push_back(successMarkEntry.first);
        }
    }
    for (const AbstractPlan* p : tr) {
        _successMarks.erase(p);
    }
}

/**
 * Clear all marks
 */
void SuccessMarks::clear()
{
    _successMarks.clear();
}

/**
 * Get all EntryPoints succeeded in the . May return nullptr.
 */
EntryPointGrp SuccessMarks::succeededEntryPoints(std::size_t parentContextHash, const AbstractPlan* p) const
{
    EntryPointGrp successEps;
    auto plan = dynamic_cast<const Plan*>(p);
    if (!plan) {
        return successEps;
    }
    for (const auto ep : plan->getEntryPoints()) {
        auto hash = contextHash(parentContextHash, ep->getId(), ep->getDynamicId());
        if (_successMarks.find(hash) != _successMarks.end()) {
            successEps.push_back(ep);
        }
    }
    return successEps;
}

/**
 * Remove all marks referring to the specified plan
 */
void SuccessMarks::removePlan(std::size_t parentContextHash, const AbstractPlan* plan)
{
    auto p = dynamic_cast<const Plan*>(plan);
    if (!p) {
        return;
    }

    for (const auto ep : p->getEntryPoints()) {
        _successMarks.erase(contextHash(parentContextHash, ep->getId(), ep->getDynamicId()));
    }
}

/**
 * Mark an EntryPoint within a plan as successfully completed
 * @param p An AbstractPlan*
 * @param e An EntryPoint*
 */
void SuccessMarks::markSuccessful(std::size_t parentContextHash, const EntryPoint* e)
{
    _successMarks.insert(contextHash(parentContextHash, e->getId(), e->getDynamicId()));
}

/**
 * Check whether an EntryPoint in a plan was completed.
 * @param p An AbstractPlan*
 * @param e An EntryPoint*
 * @return A bool
 */
bool SuccessMarks::succeeded(const AbstractPlan* p, const EntryPoint* e) const
{
    auto iter = _successMarks.find(p);
    if (iter != _successMarks.end()) {
        auto i = find(iter->second.begin(), iter->second.end(), e);
        return (i != iter->second.end());
    }
    return false;
}

/**
 * Test if at least one task has succeeded within abstract plan p
 * @param p An AbstractPlan*
 * @return A bool
 */
bool SuccessMarks::anyTaskSucceeded(const AbstractPlan* p) const
{
    auto iter = _successMarks.find(p);
    if (iter != _successMarks.end()) {
        return (!iter->second.empty());
    }
    const PlanType* pt = dynamic_cast<const PlanType*>(p);
    if (pt != nullptr) {
        for (const Plan* cp : pt->getPlans()) {
            auto iter = _successMarks.find(cp);
            if (iter != _successMarks.end() && !iter->second.empty()) {
                return true;
            }
        }
    }
    return false;
}

/**
 * Serialize to a vector of EntryPoint ids.
 */
std::vector<std::size_t> SuccessMarks::toContextGrp() const
{
    return std::vector<std::size_t>(_successMarks.begin(), _successMarks.end());
}

} /* namespace alica */
