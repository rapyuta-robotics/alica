/*
 * State.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef STATE_H_
#define STATE_H_

using namespace std;

#include <stdio.h>
#include <list>

#include "AlicaElement.h"

namespace alica
{
	class Plan;
	class Transition;
	class AbstractPlan;
	class Parametrisation;
	class EntryPoint;

	/**
	 * A State is a plan element inhabitable by agents, which contains sub-plans, sub-plantypes, and behaviours.
	 */
	class State : public AlicaElement
	{
	public:
		State();
		State(long id);
		virtual ~State();
		virtual string toString() const;
		bool isFailureState() const;
		void setFailureState(bool failureState);
		Plan* getInPlan() const;
		void setInPlan( Plan* inPlan);
		 list<Transition*>& getInTransitions() ;
		void setInTransitions(const list<Transition*>& inTransitions);
		 list<Transition*>& getOutTransitions() ;
		void setOutTransitions(list<Transition*> outTransition);
		 list<Parametrisation*>& getParametrisation();
		void setParametrisation(const list<Parametrisation*>& parametrisation);
		 list<AbstractPlan*>& getPlans() ;
		void setPlans(const list<AbstractPlan*>& plans);bool isSuccessState() const;
		void setSuccessState(bool successState);bool isTerminal() const;
		void setTerminal(bool terminal);
		EntryPoint* getEntryPoint();
		void setEntryPoint(EntryPoint* entryPoint);

	protected:
		/**
		 * The list of AbstractPlans meant to be executed in the context of this state.
		 */
		list<AbstractPlan*> plans;
		/**
		 * The list of Transitions leading to this state.
		 */
		list<Transition*> inTransitions;
		/**
		 * The list ofTransitions going from this state to another one.
		 */
		list<Transition*> outTransitions;
		/**
		 * The list of Parametrisations, which bind variables of sub-plans to variables in this state's plan.
		 */
		list<Parametrisation*> parametrisation;
		/**
		 * The plan containing this state.
		 */
		Plan* inPlan;
		/**
		 * whether or not this is a terminal state.
		 */
		bool terminal;
		/**
		 * whether or not this is a FailureState, used to avoid casting and type checking during runtime.
		 */
		bool failureState;
		/**
		 * whether or not this is a SuccessState, used to avoid casting and type checking during runtime.
		 */
		bool successState;
		/**
		 * EntryPoint of the State
		 */
		EntryPoint* entryPoint;

	};

} /* namespace Alica */

#endif /* STATE_H_ */
