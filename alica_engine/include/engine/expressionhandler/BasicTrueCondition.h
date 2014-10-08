/*
 * BasicTrueCondition.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_ENGINE_SRC_ENGINE_EXPRESSIONHANDLER_BASICTRUECONDITION_H_
#define ALICA_ALICA_ENGINE_SRC_ENGINE_EXPRESSIONHANDLER_BASICTRUECONDITION_H_

#include <engine/BasicCondition.h>

namespace alica
{

	class BasicTrueCondition : public BasicCondition
	{
	public:
		BasicTrueCondition();
		virtual ~BasicTrueCondition();

		bool evaluate(shared_ptr<RunningPlan> rp);
	};

} /* namespace alica */

#endif /* ALICA_ALICA_ENGINE_SRC_ENGINE_EXPRESSIONHANDLER_BASICTRUECONDITION_H_ */
