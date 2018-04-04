#include "engine/collections/SuccessMarks.h"

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"

namespace alica {

/**
 * Default Constructor
 */
SuccessMarks::SuccessMarks(const AlicaEngine* ae)
    : ae(ae)
{}

/**
 * Construct from a list of EntryPoint id, as received by a message
 * @param epIds A list<long>
 */
SuccessMarks::SuccessMarks(const AlicaEngine* ae, copnst list<int64_t>& epIds) 
    :   ae(ae) {

    map<long, EntryPoint*> eps = ae->getPlanRepository()->getEntryPoints();
    for (int64_t id : epIds) {
        const EntryPoint* ep;
        auto iter = eps.find(id);
        if (iter != eps.end()) {
            ep = iter->second;
            shared_ptr<list<const EntryPoint*>> s;
            auto i = this->successMarks.find(ep->getPlan());
            if (i != this->successMarks.end()) {
                s = i->second;
                if (find(s->begin(), s->end(), ep) == s->end()) {
                    s->push_back(ep);
                }
            } else {
                shared_ptr<list<EntryPoint*>> s = make_shared<list<EntryPoint*>>();
                s->push_back(ep);
                this->successMarks.insert(pair<const AbstractPlan*, shared_ptr<list<const EntryPoint*>>>(ep->getPlan(), s));
            }
        }
    }
}

SuccessMarks::~SuccessMarks() {}

/**
 * Drop every mark not occurring in plans passed as argument.
 */
void SuccessMarks::limitToPlans(const AbstractPlanSet& active) {
    list<const AbstractPlan*> tr;

    for (auto successMarkEntry : this->successMarks) {
        if (std::find(active.begin(), active.end(), successMarkEntry.first) == active.end()) {
            tr.push_back(successMarkEntry.first);
        }
    }
    for (AbstractPlan* p : tr) {
        this->successMarks.erase(p);
    }
}


/**
 * Clear all marks
 */
void SuccessMarks::clear() {
    this->successMarks.clear();
}

/**
 * Get all EntryPoints succeeded in a plan. May return nullptr.
 * @param p An AbstractPlan*
 * @return A shared_ptr<list<EntryPoint*> >
 */
shared_ptr<list<const EntryPoint*>> SuccessMarks::succeededEntryPoints(const AbstractPlan* p) const {
    //	std::cout << "SM: " << p->getName() << std::endl;
    auto successMarkEntry = this->successMarks.find(p);
    if (successMarkEntry != this->successMarks.end()) {
        return successMarkEntry->second;
    }
    return nullptr;
}

/**
 * Remove all marks referring to the specified plan.
 * @param plan An AbstractPlan*
 */
void SuccessMarks::removePlan(const AbstractPlan* plan) {
    this->successMarks.erase(plan);
}

/**
 * Mark an EntryPoint within a plan as successfully completed
 * @param p An AbstractPlan*
 * @param e An EntryPoint*
 */
void SuccessMarks::markSuccessfull(const AbstractPlan* p, const EntryPoint* e) {
    auto iter = this->successMarks.find(p);
    if (iter != this->successMarks.end()) {
        shared_ptr<list<EntryPoint*>> l = this->successMarks.at(p);
        auto i = find(l->begin(), l->end(), e);
        if (i == l->end()) {
            l->push_back(e);
        }
    } else {
        auto l = make_shared<list<EntryPoint*>>();
        l->push_back(e);
        this->successMarks.insert(pair<AbstractPlan*, shared_ptr<list<EntryPoint*>>>(p, l));
    }
}

/**
 * Check whether an EntryPoint in a plan was completed.
 * @param p An AbstractPlan*
 * @param e An EntryPoint*
 * @return A bool
 */
bool SuccessMarks::succeeded(const AbstractPlan* p, const EntryPoint* e) const {
    std::list<const EntryPoint*> l;
    auto iter = this->successMarks.find(p);
    if (iter != this->successMarks.end()) {
        l = (*iter->second);
        auto i = find(l.begin(), l.end(), e);
        return (i != l.end());
    }
    return false;
}

/**
 * Check whether an EntryPoint in a plan was completed.
 * @param planId An int
 * @param entryPointId An int
 * @return A bool
 */
bool SuccessMarks::succeeded(int64_t planId, int64_t entryPointId) const {
    const Plan* p = ae->getPlanRepository()->getPlans().at(planId);
    const EntryPoint* e = p->getEntryPoints().at(entryPointId);
    return succeeded(p, e);
}

/**
 * Test if at least one task has succeeded within abstract plan p
 * @param p An AbstractPlan*
 * @return A bool
 */
bool SuccessMarks::anyTaskSucceeded(const AbstractPlan* p) const {
    list<const EntryPoint*> l;
    auto iter = this->successMarks.find(p);
    if (iter != this->successMarks.end()) {
        l = (*iter->second);
        return (l.size() > 0);
    }
    const PlanType* pt = dynamic_cast<const PlanType*>(p);
    if (pt != nullptr) {
        for (const Plan* cp : pt->getPlans()) {
            auto iter = this->successMarks.find(cp);
            l = (*iter->second);
            if (iter != this->successMarks.end() && l.size() > 0) {
                return true;
            }
        }
    }
    return false;
}

/**
 * Serialize to a list of EntryPoint ids.
 * @return A list<long>
 */
list<int64_t> SuccessMarks::toList() const {
    list<int64_t> ret;
    for (auto pair : this->successMarks) {
        for (const EntryPoint* e : (*pair.second)) {
            ret.push_back(e->getId());
        }
    }
    return ret;
}

} /* namespace alica */
