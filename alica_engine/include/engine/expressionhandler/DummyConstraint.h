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

namespace alica
{
class ProblemDescriptor;
class RunningPlan;

class DummyConstraint : public BasicConstraint
{
    void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
};

} /* namespace alica */

#endif /* DUMMYCONSTRAINT_H_ */
