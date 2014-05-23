/*
 * PlanType.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLANTYPE_H_
#define PLANTYPE_H_

#include "AbstractPlan.h"

namespace Alica {

class PlanType: public AbstractPlan {
public:
	PlanType();
	virtual ~PlanType();
};

} /* namespace Alica */

#endif /* PLANTYPE_H_ */
