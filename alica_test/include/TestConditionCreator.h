/*
 * TestConditionCreator.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_TEST_SRC_TESTCONDITIONCREATOR_H_
#define ALICA_ALICA_TEST_SRC_TESTCONDITIONCREATOR_H_

using namespace std;

#include "engine/IConditionCreator.h"
#include <memory>
#include "iostream"

namespace alica
{
	class BasicCondition;

	class TestConditionCreator : public IConditionCreator
	{
	public:
		TestConditionCreator();
		virtual ~TestConditionCreator();
		shared_ptr<BasicCondition> createConditions(long conditionConfId);
	};

} /* namespace alica */

#endif /* ALICA_ALICA_TEST_SRC_TESTCONDITIONCREATOR_H_ */
