/*
 * Plan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLAN_H_
#define PLAN_H_

using namespace std;
#include <stddef.h>
#include <string>
#include <list>
#include <map>
#include "AbstractPlan.h"
#include "EntryPoint.h"
#include "State.h"
#include "FailureState.h"
#include "SuccessState.h"
#include "Transition.h"
#include "PostCondition.h"
#include "SyncTransition.h"

namespace alica
{

	class Plan : public AbstractPlan
	{
	public:
		Plan();
		Plan(long id);
		virtual ~Plan();
		void setFilename (string filename);
		string getFilename();
		virtual string toString() const;
		map<long, EntryPoint*>& getEntryPoints() ;
		void setEntryPoints(const map<long, EntryPoint*>& entryPoints);
		list<FailureState>& getFailureStates() ;
		void setFailureStates(const list<FailureState>& failurePoints);
		int getMaxCardinality();
		void setMaxCardinality(int maxCardinality = 0);
		int getMinCardinality() ;
		void setMinCardinality(int minCardinality = 0);
		PostCondition& getPostCondition() ;
		void setPostCondition(const PostCondition& postCondition);
		list<State>& getStates() ;
		void setStates(const list<State>& states);
		list<SuccessState>& getSuccessStates() ;
		void setSuccessStates(const list<SuccessState>& succesPoints);
		list<SyncTransition>& getSyncTransitions() ;
		void setSyncTransitions(const list<SyncTransition>& syncTransitions);
		list<Transition>& getTransitions() ;
		void setTransitions(const list<Transition>& transitions);
		EntryPoint* getEntryPointTaskID(long taskID);

	protected:
		string filename;
		int minCardinality = 0;
		int maxCardinality = 0;
		map<long, EntryPoint*> entryPoints;
		list<State> states;
		list<FailureState> failureStates;
		list<SuccessState> successStates;
		list<SyncTransition> syncTransitions;
		list<Transition> transitions;
		PostCondition postCondition;

	};

} /* namespace Alica */

#endif /* PLAN_H_ */
