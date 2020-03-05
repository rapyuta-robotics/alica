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

/**
 * Update with an IdGrp of EntryPoint ids, as received by a message
 */
void SuccessMarks::update(const AlicaEngine* ae, const IdGrp& succeededEps)
{
    clear();
    const PlanRepository::Accessor<EntryPoint>& eps = ae->getPlanRepository().getEntryPoints();
    for (int64_t id : succeededEps) {
        const EntryPoint* ep = eps.find(id);
        if (ep != nullptr) {
            auto i = _successMarks.find(ep->getPlan());
            if (i == _successMarks.end()) {
                _successMarks[ep->getPlan()] = EntryPointGrp{ep};
            } else {
                if (std::find(i->second.begin(), i->second.end(), ep) == i->second.end()) {
                    i->second.push_back(ep);
                }
            }
        }
    }
}

SuccessMarks::~SuccessMarks() {}

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
 * Get all EntryPoints succeeded in a plan. May return nullptr.
 * @param p An AbstractPlan*
 * @return A shared_ptr<list<EntryPoint*> >
 */
const EntryPointGrp* SuccessMarks::succeededEntryPoints(const AbstractPlan* p) const
{
    auto successMarkEntry = _successMarks.find(p);
    if (successMarkEntry != _successMarks.end()) {
        return &successMarkEntry->second;
    }
    return nullptr;
}

/**
 * Remove all marks referring to the specified plan.
 * @param plan An AbstractPlan*
 */
void SuccessMarks::removePlan(const AbstractPlan* plan)
{
    _successMarks.erase(plan);
}

/**
 * Mark an EntryPoint within a plan as successfully completed
 * @param p An AbstractPlan*
 * @param e An EntryPoint*
 */
void SuccessMarks::markSuccessfull(const AbstractPlan* p, const EntryPoint* e)
{
    EntryPointGrp& l = _successMarks[p];
    auto i = std::find(l.begin(), l.end(), e);
    if (i == l.end()) {
        l.push_back(e);
    }
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
IdGrp SuccessMarks::toIdGrp() const
{
    IdGrp ret;
    for (const auto& pair : _successMarks) {
        for (const EntryPoint* e : pair.second) {
            ret.push_back(e->getId());
        }
    }
    return ret;
}

} /* namespace alica */
