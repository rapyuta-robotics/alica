/*
 * Plan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef PLAN_H_
#define PLAN_H_

#include "AbstractPlan.h"

namespace alica
{

	class Plan : public AbstractPlan
	{
	public:
		Plan();
		virtual ~Plan();
	};

} /* namespace Alica */

#endif /* PLAN_H_ */
