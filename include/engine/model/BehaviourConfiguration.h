/*
 * BehaviourConfiguration.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef BEHAVIOURCONFIGURATION_H_
#define BEHAVIOURCONFIGURATION_H_

#include "AbstractPlan.h"

namespace Alica {

class BehaviourConfiguration: public AbstractPlan {
public:
	BehaviourConfiguration();
	virtual ~BehaviourConfiguration();
};

} /* namespace Alica */

#endif /* BEHAVIOURCONFIGURATION_H_ */
