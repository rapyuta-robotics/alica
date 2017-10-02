#pragma once

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
