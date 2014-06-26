/*
 * SuccessMarks.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: Stefan Jakob
 */

#include <engine/collections/SuccessMarks.h>

#include <engine/model/EntryPoint.h>
#include <engine/model/PlanType.h>
#include <engine/model/AbstractPlan.h>
#include <engine/model/Plan.h>
#include <engine/AlicaEngine.h>
#include <engine/PlanRepository.h>

namespace alica
{

	SuccessMarks::SuccessMarks()
	{
		// TODO Auto-generated constructor stub

	}

	SuccessMarks::~SuccessMarks()
	{
		// TODO Auto-generated destructor stub
	}

	void SuccessMarks::limitToPlans(unique_ptr<unordered_set<AbstractPlan*> > active)
	{
		list<AbstractPlan*> tr;
		for (map<AbstractPlan*, shared_ptr<list<EntryPoint*> > >::const_iterator iterator =
				this->getSuccessMarks().begin(); iterator != this->getSuccessMarks().end(); iterator++)
		{
			if (active->find(iterator->first) != active->end())
			{
				tr.push_back(iterator->first);
			}
		}
		for (AbstractPlan* p : tr)
		{
			this->getSuccessMarks().erase(p);
		}
	}

	map<AbstractPlan*, shared_ptr<list<EntryPoint*> > > SuccessMarks::getSuccessMarks() const
	{
		return succesMarks;
	}

	void SuccessMarks::setSuccesMarks(map<AbstractPlan*, shared_ptr<list<EntryPoint*> > > succesMarks)
	{
		this->succesMarks = succesMarks;
	}

	void SuccessMarks::clear()
	{
		this->succesMarks.clear();
	}

	shared_ptr<list<EntryPoint*> > SuccessMarks::succeededEntryPoints(AbstractPlan* p)
	{
		for (map<AbstractPlan*, shared_ptr<list<EntryPoint*> > >::const_iterator iterator =
				this->getSuccessMarks().begin(); iterator != this->getSuccessMarks().end(); iterator++)
		{
			if (iterator->first == p)
			{
				return iterator->second;
			}
		}
		return nullptr;
	}

	SuccessMarks::SuccessMarks(list<long> epIds)
	{
		map<long, EntryPoint*> eps = AlicaEngine::getInstance()->getPlanRepository()->getEntryPoints();
		for (long id : epIds)
		{
			EntryPoint* ep;
			auto iter = eps.find(id);
			if (iter != eps.end())
			{
				ep = iter->second;
				list<EntryPoint*> s;
				auto i = this->getSuccessMarks().find(ep->getPlan());
				if (i != this->getSuccessMarks().end())
				{
					s = (*i->second);
					if (find(s.begin(), s.end(), ep) == s.end())
					{
						s.push_back(ep);
					}
					else
					{
						shared_ptr<list<EntryPoint*> > s;
						s->push_back(ep);
						this->getSuccessMarks().insert(pair<AbstractPlan*, shared_ptr<list<EntryPoint*> > >(ep->getPlan(),s));
					}
				}
			}
		}
	}

	void SuccessMarks::removePlan(AbstractPlan* plan)
	{
		this->getSuccessMarks().erase(plan);
	}

	void SuccessMarks::markSuccessfull(AbstractPlan* p, EntryPoint* e)
	{
		auto iter = this->getSuccessMarks().find(p);
		if (iter != this->getSuccessMarks().end())
		{
			shared_ptr<list<EntryPoint*> > l = this->getSuccessMarks().at(p);
			auto i = find(l->begin(), l->end(), e);
			if (i == l->end())
			{
				l->push_back(e);
			}
		}
		else
		{
			shared_ptr<list<EntryPoint*> > l;
			l->push_back(e);
			this->getSuccessMarks().insert(pair<AbstractPlan*, shared_ptr<list<EntryPoint*> > >(p, l));

		}
	}

	bool SuccessMarks::succeeded(AbstractPlan* p, EntryPoint* e)
	{
		list<EntryPoint*> l;
		auto iter = this->getSuccessMarks().find(p);
		if (iter != this->getSuccessMarks().end())
		{
			l = (*iter->second);
			auto i = find(l.begin(), l.end(), e);
			return (i != l.end());
		}
		return false;
	}

	bool SuccessMarks::succeeded(long planId, long entryPointId)
	{
		Plan* p = AlicaEngine::getInstance()->getPlanRepository()->getPlans().at(planId);
		EntryPoint* e = p->getEntryPoints().at(entryPointId);
		return succeeded(p, e);
	}

	bool SuccessMarks::anyTaskSucceeded(AbstractPlan* p)
	{
		list<EntryPoint*> l;
		auto iter = this->getSuccessMarks().find(p);
		if (iter != this->getSuccessMarks().end())
		{
			l = (*iter->second);
			return (l.size() > 0);
		}
		PlanType* pt = dynamic_cast<PlanType*>(p);
		if (pt != nullptr)
		{
			for (Plan* cp : pt->getPlans())
			{
				auto iter = this->getSuccessMarks().find(cp);
				if (iter != this->getSuccessMarks().end() && l.size() > 0)
				{
					return true;
				}
			}
		}
		return false;
	}

	list<long> SuccessMarks::toList()
	{
		list<long> ret;
		for (auto pair : this->getSuccessMarks())
		{
			for (EntryPoint* e : (*pair.second))
			{
				ret.push_back(e->getId());
			}

		}
		return ret;
	}

} /* namespace alica */

