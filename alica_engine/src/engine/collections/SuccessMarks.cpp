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
SuccessMarks::SuccessMarks(const AlicaEngine *ae)
{
    this->ae = ae;
}

/**
 * Construct from a list of EntryPoint id, as received by a message
 * @param epIds A list<long>
 */
SuccessMarks::SuccessMarks(const AlicaEngine *ae, list<long> epIds)
{
    this->ae = ae;
    map<long, EntryPoint *> eps = ae->getPlanRepository()->getEntryPoints();
    for (long id : epIds)
    {
        EntryPoint *ep;
        auto iter = eps.find(id);
        if (iter != eps.end())
        {
            ep = iter->second;
            shared_ptr<list<EntryPoint *>> s;
            auto i = this->successMarks.find(ep->getPlan());
            if (i != this->successMarks.end())
            {
                s = i->second;
                if (find(s->begin(), s->end(), ep) == s->end())
                {
                    s->push_back(ep);
                }
            }
            else
            {
                shared_ptr<list<EntryPoint *>> s = make_shared<list<EntryPoint *>>();
                s->push_back(ep);
                this->successMarks.insert(pair<AbstractPlan *, shared_ptr<list<EntryPoint *>>>(ep->getPlan(), s));
            }
        }
    }
}

SuccessMarks::~SuccessMarks()
{
}

/**
 * Drop every mark not occurring in plans passed as argument.
 * @param active An unique_ptr<unordered_set<AbstractPlan*> >
 */
void SuccessMarks::limitToPlans(unique_ptr<unordered_set<AbstractPlan *>> active)
{
    list<AbstractPlan *> tr;
    for (auto successMarkEntry : this->successMarks)
    {
        if (active->find(successMarkEntry.first) == active->end())
        {
            tr.push_back(successMarkEntry.first);
        }
    }
    for (AbstractPlan *p : tr)
    {
        this->successMarks.erase(p);
    }
}

map<AbstractPlan *, shared_ptr<list<EntryPoint *>>> &SuccessMarks::getSuccessMarks()
{
    return successMarks;
}

void SuccessMarks::setSuccessMarks(map<AbstractPlan *, shared_ptr<list<EntryPoint *>>> successMarks)
{
    this->successMarks = successMarks;
}

/**
 * Clear all marks
 */
void SuccessMarks::clear()
{
    this->successMarks.clear();
}

/**
 * Get all EntryPoints succeeded in a plan. May return nullptr.
 * @param p An AbstractPlan*
 * @return A shared_ptr<list<EntryPoint*> >
 */
shared_ptr<list<EntryPoint *>> SuccessMarks::succeededEntryPoints(AbstractPlan *p) const
{
	auto successMarkEntry = this->successMarks.find(p);
	if (successMarkEntry != this->successMarks.end())
	{
		return successMarkEntry->second;
	}
    return nullptr;
}

/**
 * Remove all marks referring to the specified plan.
 * @param plan An AbstractPlan*
 */
void SuccessMarks::removePlan(AbstractPlan *plan)
{
    this->successMarks.erase(plan);
}

/**
 * Mark an EntryPoint within a plan as successfully completed
 * @param p An AbstractPlan*
 * @param e An EntryPoint*
 */
void SuccessMarks::markSuccessfull(AbstractPlan *p, EntryPoint *e)
{
    auto iter = this->successMarks.find(p);
    if (iter != this->successMarks.end())
    {
        shared_ptr<list<EntryPoint *>> l = this->successMarks.at(p);
        auto i = find(l->begin(), l->end(), e);
        if (i == l->end())
        {
            l->push_back(e);
        }
    }
    else
    {
        auto l = make_shared<list<EntryPoint *>>();
        l->push_back(e);
        this->successMarks.insert(pair<AbstractPlan *, shared_ptr<list<EntryPoint *>>>(p, l));
    }
}

/**
 * Check whether an EntryPoint in a plan was completed.
 * @param p An AbstractPlan*
 * @param e An EntryPoint*
 * @return A bool
 */
bool SuccessMarks::succeeded(AbstractPlan *p, EntryPoint *e)
{
    list<EntryPoint *> l;
    auto iter = this->successMarks.find(p);
    if (iter != this->successMarks.end())
    {
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
bool SuccessMarks::succeeded(long planId, long entryPointId)
{
    Plan *p = ae->getPlanRepository()->getPlans().at(planId);
    EntryPoint *e = p->getEntryPoints().at(entryPointId);
    return succeeded(p, e);
}

/**
 * Test if at least one task has succeeded within abstract plan p
 * @param p An AbstractPlan*
 * @return A bool
 */
bool SuccessMarks::anyTaskSucceeded(AbstractPlan *p)
{
    list<EntryPoint *> l;
    auto iter = this->successMarks.find(p);
    if (iter != this->successMarks.end())
    {
        l = (*iter->second);
        return (l.size() > 0);
    }
    PlanType *pt = dynamic_cast<PlanType *>(p);
    if (pt != nullptr)
    {
        for (Plan *cp : pt->getPlans())
        {
            auto iter = this->successMarks.find(cp);
            l = (*iter->second);
            if (iter != this->successMarks.end() && l.size() > 0)
            {
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
list<long> SuccessMarks::toList() const
{
    list<long> ret;
    for (auto pair : this->successMarks)
    {
        for (EntryPoint *e : (*pair.second))
        {
            ret.push_back(e->getId());
        }
    }
    return ret;
}

} /* namespace alica */
