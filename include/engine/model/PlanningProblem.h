/*
 * PlanningProblem.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLANNINGPROBLEM_H_
#define PLANNINGPROBLEM_H_

#include "AbstractPlan.h"

namespace alica
{

class PlanningProblem : public AbstractPlan
{
public:
	PlanningProblem();
	virtual ~PlanningProblem();
};

} /* namespace Alica */

#endif /* PLANNINGPROBLEM_H_ */
