/*
 * AbstractPlan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ABSTRACTPLAN_H_
#define ABSTRACTPLAN_H_

#include "AlicaElement.h"

namespace alica
{

	class AbstractPlan : public AlicaElement
	{
	public:
		AbstractPlan();
		virtual ~AbstractPlan();
	};

} /* namespace Alica */

#endif /* ABSTRACTPLAN_H_ */
