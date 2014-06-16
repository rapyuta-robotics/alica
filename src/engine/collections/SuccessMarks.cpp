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
		for(map<AbstractPlan*, list<EntryPoint*> >::const_iterator iterator = this->getSuccesMarks().begin(); iterator != this->getSuccesMarks().end(); iterator ++)
		{
			if(active.get()->find(iterator->first) != active.get()->end())
			{
				tr.push_back(iterator->first);
			}
		}
		for(AbstractPlan* p : tr)
		{
			this->getSuccesMarks().erase(p);
		}
	}


	 map<AbstractPlan*, list<EntryPoint*> > SuccessMarks::getSuccesMarks() const
	{
		 return succesMarks;
	}

	void SuccessMarks::setSuccesMarks(map<AbstractPlan*, list<EntryPoint*> > succesMarks)
	{
	this->succesMarks = succesMarks;
	}

	void SuccessMarks::clear()
	{
		this->succesMarks.clear();
	}

} /* namespace alica */


