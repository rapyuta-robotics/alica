/*
 * SuccessMarks.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: stefan
 */

#include <engine/collections/SuccessMarks.h>

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
			if(iterator->first == p)
			{
				return iterator->second;
			}
		}
		return nullptr;
	}

} /* namespace alica */

