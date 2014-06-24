/*
 * IBehaviourPool.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stephan Opfer
 */

#ifndef IBEHAVIOURPOOL_H_
#define IBEHAVIOURPOOL_H_

using namespace std;

namespace alica
{
	class RunningPlan;
	class Behaviour;
	class BasicBehaviour;
	class IBehaviourCreator;

	class IBehaviourPool
	{
	public:
		virtual ~IBehaviourPool()
		{
		}

		// TODO find a nice way to trigger
		// C#: public delegate void EngineTrigger(object arg);


		virtual void startBehaviour(RunningPlan rp) = 0;

		virtual void stopBehaviour(RunningPlan rp) = 0;

		virtual bool init(IBehaviourCreator* bc) = 0;

		virtual void stopAll() = 0;
	};

} /* namespace alica */

#endif /* IBEHAVIOURPOOL_H_ */
