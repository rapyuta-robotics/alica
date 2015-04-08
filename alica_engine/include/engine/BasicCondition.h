/*
 * BasicCondition.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_ENGINE_INCLUDE_ENGINE_BASICCONDITION_H_
#define ALICA_ALICA_ENGINE_INCLUDE_ENGINE_BASICCONDITION_H_

#include <memory>

using namespace std;

namespace alica
{

	class RunningPlan;

	class BasicCondition
	{
	public:
		BasicCondition();
		virtual ~BasicCondition();
		virtual bool evaluate(shared_ptr<RunningPlan> rp) = 0;

		bool isStateTimedOut(unsigned long timeOut, shared_ptr<RunningPlan> rp);
		bool isTimeOut(unsigned long timeOut, unsigned long startTime, shared_ptr<RunningPlan> rp);

	};

} /* namespace alica */

#endif /* ALICA_ALICA_ENGINE_INCLUDE_ENGINE_BASICCONDITION_H_ */
