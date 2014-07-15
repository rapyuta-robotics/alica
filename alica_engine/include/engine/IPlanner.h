/*
 * IPlanner.h
 *
 *  Created on: Jul 15, 2014
 *      Author: Stefan Jakob
 */

#ifndef IPLANNER_H_
#define IPLANNER_H_

namespace alica
{

	class Plan;

	class IPlanner
	{
	public:
		virtual ~IPlanner() {}

		virtual Plan* requestPlan(PlanningProblem* pp) = 0;
	};

} /* namespace alica */

#endif /* IPLANNER_H_ */
