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

	class EntryPoint : public AlicaElement
	{
	public:
		EntryPoint();
		virtual ~EntryPoint();

		const long IDLEID = -1;
		void computeReachabilitySet();
		string toString ();
		static bool compareTo(const EntryPoint* ep1 , const EntryPoint* ep2);

		const Task* getTask() const;
		void setTask(const Task* task);
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
		State* state;
		const Task* task;
		Plan* plan;
		int minCardinality = 0;
		int maxCardinality = 0;
		bool successRequired;
		unordered_set<State*> reachableStates;

	};

} /* namespace Alica */

#endif /* ENTRYPOINT_H_ */
