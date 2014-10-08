/*
 * BasicFalseExpression.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_ENGINE_SRC_ENGINE_EXPRESSIONHANDLER_BASICFALSECONDITION_H_
#define ALICA_ALICA_ENGINE_SRC_ENGINE_EXPRESSIONHANDLER_BASICFALSECONDITION_H_

#include <engine/BasicCondition.h>

namespace alica
{

	class BasicFalseCondition : public BasicCondition
	{
	public:
		BasicFalseCondition();
		virtual ~BasicFalseCondition();

		bool evaluate(shared_ptr<RunningPlan> rp);
	};

} /* namespace alica */

#endif /* ALICA_ALICA_ENGINE_SRC_ENGINE_EXPRESSIONHANDLER_BASICFALSECONDITION_H_ */
