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

#include "AlicaElement.h"
#include "Task.h"
#include "State.h"

namespace alica
{
	class Plan;

	class EntryPoint : public AlicaElement
	{
	public:
		EntryPoint();
		virtual ~EntryPoint();

		const long IDLEID = -1;

		const Task* getTask() const;
		void setTask(const Task* task);
		const Plan* getPlan() const;
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

	private:


	protected:
		const Task* task;
		const Plan* plan;
		int minCardinality = 0;
		int maxCardinality = 0;
		bool successRequired;
		unordered_set<State*> reachableStates;

	};

} /* namespace Alica */

#endif /* ENTRYPOINT_H_ */
