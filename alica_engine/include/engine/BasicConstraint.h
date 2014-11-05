/*
 * BasicContraint.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_ENGINE_SRC_ENGINE_BASICCONTRAINT_H_
#define ALICA_ALICA_ENGINE_SRC_ENGINE_BASICCONTRAINT_H_

#include <memory>

using namespace std;

namespace alica
{
	class ConstraintDescriptor;
	class RunningPlan;

	class BasicConstraint
	{
	public:
		virtual ~BasicConstraint(){}

		virtual void getConstraint(shared_ptr<ConstraintDescriptor> c, shared_ptr<RunningPlan> rp) = 0;
	};

} /* namespace alica */

#endif /* ALICA_ALICA_ENGINE_SRC_ENGINE_BASICCONTRAINT_H_ */
