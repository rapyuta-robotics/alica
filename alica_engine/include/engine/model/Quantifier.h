/*
 * Quantifier.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef QUANTIFIER_H_
#define QUANTIFIER_H_

#include "AlicaElement.h"

#include "engine/IRobotID.h"

#include <list>
#include <string>
#include <typeinfo>
#include <vector>
#include <memory>


using namespace std;
namespace alica
{
	class State;
	class EntryPoint;
	class Plan;
	class Variable;
	class RunningPlan;
	class AlicaEngine;
	class SolverTerm;

	/**
	 * A quantifier encapsulates a set of Variables, belonging to a domain artifact, scoped under a AlicaElement
	 */
	class Quantifier : public AlicaElement
	{
	public:
		Quantifier(long id = 0);
		virtual ~Quantifier();
		list<string>& getDomainIdentifiers() ;
		void setDomainIdentifiers(const list<string>& domainIdentifiers);
		bool isScopeIsEntryPoint() const;
		bool isScopeIsPlan() const;
		bool isScopeIsState() const;
		State* getScopedState();
		EntryPoint* getScopedEntryPoint();
		Plan* getScopedPlan();
		void setScope(AlicaEngine* a,AlicaElement* ae);
		AlicaElement* getScope();
		/**
		 * Access the list of sorted Variables under the scope of this quantifier given a runningplan.
		 * @param p A RunningPlan
		 * @param agentsInScope A shared_ptr<vector<int> >
		 * @return A shared_ptr<list<vector<Variable* > > >
		 */
		virtual shared_ptr<list<vector<const Variable* > > >getDomainVariables(shared_ptr<RunningPlan>& p, shared_ptr<vector<const alica::IRobotID*> >& agentsInScope) = 0;
//		/**
//		 * Access the list of sorted AD.Terms under the scope of this quantifier given a RunningPlan.
//		 * @param agentsInScope A shared_ptr<vector<int> >
//		 * @return A shared_ptr<list<vector<shared_ptr<SolverTerm> > > >
//		 */
//		virtual shared_ptr<list<vector<shared_ptr<SolverTerm>> > > getSortedTerms(RunningPlan* p, shared_ptr<vector<int> > agentsInScope) = 0;

	private:
		list<string> domainIdentifiers;
		void setScopeIsEntryPoint(bool scopeIsEntryPoint);
		void setScopeIsPlan(bool scopeIsPlan);
		void setScopeIsState(bool scopeIsState);

	protected:
		/**
		 * Indicates that the scope of this quantifier is an EntryPoint
		 */
		bool scopeIsEntryPoint;
		/**
		 * Indicates that the scope of this quantifier is a Plan
		 */
		bool scopeIsPlan;
		/**
		 * Indicates that the scope of this quantifier is an State
		 */
		bool scopeIsState;
		EntryPoint* entryPoint;
		State* state;
		Plan* plan;
	};

} /* namespace Alica */

#endif /* QUANTIFIER_H_ */
