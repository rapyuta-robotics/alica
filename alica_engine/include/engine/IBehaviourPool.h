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
	class BehaviourConfiguration;

	class IBehaviourPool
	{
	public:
		virtual ~IBehaviourPool()
		{
		}

		/**
		 * An engine trigger is a delegate used to call eventDriven BasicBehaviours. Events of this type can be used by a BasicBehaviour to manage when it is called.
		 */
		// TODO find a nice way to trigger
		// C#: public delegate void EngineTrigger(object arg);

		/**
		 * Destroys a behaviour represented by its RunningPlan.
		 * @param rp A RunningPlan
		 */
		virtual void stopBehaviour(shared_ptr<RunningPlan> rp) = 0;

		/**
		 * Starts a behaviour represented by its RunningPlan from the set of currently active behaviour.
		 * @param rp A RunningPlan
		 */
		virtual void startBehaviour(shared_ptr<RunningPlan> rp) = 0;

		/**
		 * Initialises this Engine Module
		 */
		virtual bool init(IBehaviourCreator* bc) = 0;

		/**
		 * Stops this engine module
		 */
		virtual void stopAll() = 0;

		/**
		 * Returns a map of all available behaviours
		 */
		virtual map<BehaviourConfiguration*, shared_ptr<BasicBehaviour> >* getAvailableBehaviours() = 0;
	};

} /* namespace alica */

#endif /* IBEHAVIOURPOOL_H_ */
