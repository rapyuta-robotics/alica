#pragma once

#include <memory>

namespace alica
{
	class ProblemDescriptor;
	class RunningPlan;

	class BasicConstraint
	{
	public:
		virtual ~BasicConstraint(){}

		virtual void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) = 0;
	};

} /* namespace alica */
