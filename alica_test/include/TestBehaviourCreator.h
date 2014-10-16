/*
 * TestBehaviourCreator.h
 *
 *  Created on: Jun 18, 2014
 *      Author: Stephan Opfer
 */

#ifndef TESTBEHAVIOURCREATOR_H_
#define TESTBEHAVIOURCREATOR_H_

#include <engine/IBehaviourCreator.h>

#include <memory>
#include <iostream>

namespace alicaTests
{

	class BasicBehaviour;

	class TestBehaviourCreator : public alica::IBehaviourCreator
	{
	public:
		TestBehaviourCreator();
		virtual ~TestBehaviourCreator();
		virtual shared_ptr<alica::BasicBehaviour> createBehaviour(long behaviourConfId);
	};

} /* namespace alica */

#endif /* TESTBEHAVIOURCREATOR_H_ */
