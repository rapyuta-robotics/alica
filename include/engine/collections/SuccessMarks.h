/*
 * SuccessMarks.h
 *
 *  Created on: Jun 16, 2014
 *      Author: stefan
 */

#ifndef SUCCESSMARKS_H_
#define SUCCESSMARKS_H_

using namespace std;

#include <map>
#include <memory>
#include <unordered_set>
#include <list>
#include <algorithm>
#include <memory>

namespace alica
{

	class AbstractPlan;
	class EntryPoint;

	class SuccessMarks
	{
	public:
		SuccessMarks();
		virtual ~SuccessMarks();


		//TODO uses ICollection in C# so far only unordered_set needed
		void limitToPlans(unique_ptr<unordered_set<AbstractPlan*> > active);
		map<AbstractPlan*,shared_ptr<list<EntryPoint*> > > getSuccessMarks() const;
		void setSuccesMarks(map<AbstractPlan*,shared_ptr<list<EntryPoint*> > > succesMarks);
		void clear();
		shared_ptr<list<EntryPoint*> >succeededEntryPoints(AbstractPlan* p);

	protected:
		map<AbstractPlan*,shared_ptr<list<EntryPoint*> > > succesMarks;
	};

} /* namespace alica */

#endif /* SUCCESSMARKS_H_ */
