/*
 * TestUtilityFunctionCreator.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_TEST_SRC_TESTUTILITYFUNCTIONCREATOR_H_
#define ALICA_ALICA_TEST_SRC_TESTUTILITYFUNCTIONCREATOR_H_

#include <engine/IUtilityCreator.h>

namespace alica
{

	class TestUtilityFunctionCreator : public IUtilityCreator
	{
	public:
		virtual ~TestUtilityFunctionCreator();
		TestUtilityFunctionCreator();
	};

} /* namespace alica */

#endif /* ALICA_ALICA_TEST_SRC_TESTUTILITYFUNCTIONCREATOR_H_ */
