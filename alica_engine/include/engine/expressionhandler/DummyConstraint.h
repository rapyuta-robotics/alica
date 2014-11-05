/*
 * DummyConstraint.h
 *
 *  Created on: Oct 23, 2014
 *      Author: Philipp
 */

#ifndef DUMMYCONSTRAINT_H_
#define DUMMYCONSTRAINT_H_

#include "engine/BasicConstraint.h"
#include <memory>

using namespace std;

namespace alica
{
	class ConstraintDescriptor;
	class RunningPlan;

	class DummyConstraint : public BasicConstraint
	{
		void getConstraint(shared_ptr<ConstraintDescriptor> c, shared_ptr<RunningPlan> rp);
	};

} /* namespace alica */

#endif /* DUMMYCONSTRAINT_H_ */
