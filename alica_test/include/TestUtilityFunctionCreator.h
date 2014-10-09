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

namespace alica
{
	class BasicUtilityFunction;

	class TestUtilityFunctionCreator : public IUtilityCreator
	{
	public:
		virtual ~TestUtilityFunctionCreator();
		TestUtilityFunctionCreator();
		shared_ptr<BasicUtilityFunction> createUtility(long utilityfunctionConfId);
	};

} /* namespace alica */

#endif /* ALICA_ALICA_TEST_SRC_TESTUTILITYFUNCTIONCREATOR_H_ */
