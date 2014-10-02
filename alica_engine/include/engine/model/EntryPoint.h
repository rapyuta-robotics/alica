/*
 * EntryPoint.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ENTRYPOINT_H_
#define ENTRYPOINT_H_

using namespace std;

#include <unordered_set>
#include <string>
#include <sstream>
#include <list>

#include "AlicaElement.h"

namespace alica
{

	class Plan;
	class State;
	class Task;

	/**
	 * An EntryPoint is used to identify the initial state of a task within a plan.
	 * It also holds cardinalities and any information specific to this (task,plan) tuple.
	 */
	class EntryPoint : public AlicaElement
	{
	public:
		EntryPoint();
		virtual ~EntryPoint();

		/**
		 * A value encoding the do-nothing task used in loosely coupled task allocation.
		 */
		const static long IDLEID = -1;// For Idle EntryPoint...
		void computeReachabilitySet();
		string toString ();
		static bool compareTo(EntryPoint* ep1 , EntryPoint* ep2);

		Task* getTask();
		void setTask(Task* task);
		Plan* getPlan() const;
		void setPlan(Plan* plan);
		const int getMaxCardinality() const;
		void setMaxCardinality(int maxCardinality = 0);
		const int getMinCardinality() const;
		void setMinCardinality(int minCardinality = 0);
		void setSuccessRequired(bool successRequired);
		const bool getSuccessRequired() const;
		bool isSuccessRequired() const;
		const unordered_set<State*>& getReachableStates() const;
		void setReachableStates(const unordered_set<State*>& reachableStates);
		State* getState();
		void setState(State* state);

	protected:
		/**
		 * The initial state of this entrypoint's task.
		 */
		State* state;
		/**
		 * The task of this entrypoint.
		 */
		Task* task;
		/**
		 * The plan to which this entrypoint belongs.
		 */
		Plan* plan;
		/**
		 * The minimum amount of agents required to execute this entrypoint's task within Plan.
		 */
		int minCardinality = 0;
		/**
		 * The maximum amount of agents allowed to execute this entrypoint's task within Plan.
		 */
		int maxCardinality = 0;
		/**
		 * whether or not a success of this task is required for Plan to be successful. Otherwise, this task is optional.
		 */
		bool successRequired;
		/**
		 * The set of states reachable from the initial state.
		 */
		unordered_set<State*> reachableStates;

	};

} /* namespace Alica */

#endif /* ENTRYPOINT_H_ */
