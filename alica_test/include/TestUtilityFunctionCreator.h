/*
 * TestUtilityFunctionCreator.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#ifndef ALICA_ALICA_TEST_SRC_TESTUTILITYFUNCTIONCREATOR_H_
#define ALICA_ALICA_TEST_SRC_TESTUTILITYFUNCTIONCREATOR_H_

#include <engine/IUtilityCreator.h>
#include <memory>

using namespace std;

namespace alicaTests
{
	class BasicUtilityFunction;

	class TestUtilityFunctionCreator : public alica::IUtilityCreator
	{
	public:
		virtual ~TestUtilityFunctionCreator();
		TestUtilityFunctionCreator();
		shared_ptr<alica::BasicUtilityFunction> createUtility(long utilityfunctionConfId);
	};

} /* namespace alica */

#endif /* ALICA_ALICA_TEST_SRC_TESTUTILITYFUNCTIONCREATOR_H_ */
