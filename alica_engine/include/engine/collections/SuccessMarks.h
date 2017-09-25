#pragma once

#include <map>
#include <memory>
#include <unordered_set>
#include <list>
#include <algorithm>

namespace alica
{

	class AbstractPlan;
	class EntryPoint;
	class AlicaEngine;

	/**
	 * Globally holds information about succeeded entrypoints for a specific robot
	 */
	class SuccessMarks
	{
	public:
		SuccessMarks(const AlicaEngine* ae);
		SuccessMarks(const AlicaEngine* ae, std::list<long> epIds);
		virtual ~SuccessMarks();

		//TODO uses ICollection in C# so far only unordered_set needed
		void limitToPlans(std::unique_ptr<std::unordered_set<AbstractPlan*> > active);
		std::map<AbstractPlan*,std::shared_ptr<std::list<EntryPoint*> > >& getSuccessMarks();
		void setSuccesMarks(std::map<AbstractPlan*,std::shared_ptr<std::list<EntryPoint*> > > succesMarks);
		void clear();
		std::shared_ptr<std::list<EntryPoint*> >succeededEntryPoints(AbstractPlan* p);
		void removePlan(AbstractPlan* plan);
		void markSuccessfull(AbstractPlan* p, EntryPoint* e);
		bool succeeded(AbstractPlan* p, EntryPoint* e);
		bool succeeded(long planId, long entryPointId);
		bool anyTaskSucceeded(AbstractPlan* p);
		std::list<long> toList();

	protected:
		std::map<AbstractPlan*,std::shared_ptr<std::list<EntryPoint*> > > succesMarks;
		const AlicaEngine* ae;
	};

} /* namespace alica */
