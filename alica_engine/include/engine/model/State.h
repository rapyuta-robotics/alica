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
		list<AbstractPlan*> plans;
		list<Transition*> inTransitions;
		list<Transition*> outTransitions;
		list<Parametrisation*> parametrisation;
		Plan* inPlan;
		bool terminal;
		bool failureState;
		bool successState;
		EntryPoint* entryPoint;

	};

} /* namespace Alica */

#endif /* STATE_H_ */
