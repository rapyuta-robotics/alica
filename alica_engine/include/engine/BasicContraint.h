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

	class BasicContraint
	{
	public:
		BasicContraint();
		virtual ~BasicContraint();

		virtual void getConstraint(shared_ptr<ConstraintDescriptor> c, shared_ptr<RunningPlan>) = 0;
	};

} /* namespace alica */

#endif /* ALICA_ALICA_ENGINE_SRC_ENGINE_BASICCONTRAINT_H_ */
