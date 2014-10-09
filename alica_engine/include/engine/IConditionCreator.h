/*
 * IConditionCreator.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_ENGINE_INCLUDE_ENGINE_ICONDITIONCREATOR_H_
#define ALICA_ALICA_ENGINE_INCLUDE_ENGINE_ICONDITIONCREATOR_H_

#include "engine/BasicCondition.h"

using namespace std;

namespace alica
{
	class BasicCondition;

	class IConditionCreator
	{
	public:
		virtual ~IConditionCreator() {}
		virtual shared_ptr<BasicCondition> createConditions(long conditionConfId) = 0;
	};

} /* namespace alica */

#endif /* ALICA_ALICA_ENGINE_INCLUDE_ENGINE_ICONDITIONCREATOR_H_ */
