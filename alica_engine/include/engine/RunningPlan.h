/*
 * RunningPlan.h
 *
 *  Created on: Jun 10, 2014
 *      Author: Stefan Jakob
 */

#ifndef RUNNINGPLAN_H_
#define RUNNINGPLAN_H_

using namespace std;

#include <list>
#include <memory>
#include <iostream>

namespace alica
{

	class BasicBehaviour;
	class AbstractPlan;

	class RunningPlan
	{
	public:
		RunningPlan();
		virtual ~RunningPlan();
		bool isBehaviour() const;
		void setBehaviour(bool behaviour);
		const list<RunningPlan*>& getChildren() const;
		void setChildren(const list<RunningPlan*>& children);
		AbstractPlan* getPlan() const;
		void setPlan(AbstractPlan* plan);
		shared_ptr<BasicBehaviour> getBasicBehaviour();
		void setBasicBehaviour(shared_ptr<BasicBehaviour> basicBehaviour);

		void printRecursive();

	protected:
		bool behaviour;
		AbstractPlan* plan;
		shared_ptr<BasicBehaviour> basicBehaviour;
		list<RunningPlan*> children;
	};

} /* namespace alica */

#endif /* RUNNINGPLAN_H_ */
