/*
 * DummyBehaviourCreator.h
 *
 *  Created on: Jun 18, 2014
 *      Author: Stephan Opfer
 */

#ifndef DUMMYBEHAVIOURCREATOR_H_
#define DUMMYBEHAVIOURCREATOR_H_

#include <engine/IBehaviourCreator.h>

#include <memory>
#include <iostream>

namespace alica
{

	class BasicBehaviour;

	class DummyBehaviourCreator : public IBehaviourCreator
	{
	public:
		DummyBehaviourCreator();
		virtual ~DummyBehaviourCreator();
		virtual unique_ptr<BasicBehaviour> createBehaviour(string behaviourName);
	};

} /* namespace alica */

#endif /* DUMMYBEHAVIOURCREATOR_H_ */
