/*
 * BehaviourPool.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stephan Opfer
 */

#ifndef BEHAVIOURPOOL_H_
#define BEHAVIOURPOOL_H_

using namespace std;

#include <map>
#include <typeindex>
#include <iostream>

#include <engine/IBehaviourPool.h>

namespace alica
{

	class BasicBehaviour;

	class BehaviourPool : public IBehaviourPool
	{

	public:
		BehaviourPool();
		virtual ~BehaviourPool();
		void stop();
		void init();
		bool isBehaviourAvailable(const Behaviour* b) const;
		void removeBehaviour(RunningPlan rp);
		void addBehaviour(RunningPlan rp);
		void registerBehaviour(Behaviour* behaviour, createFunc createFunction);

	private:
		/**
		 * This map matches behaviours to their create methods.
		 */
		map<Behaviour*, BasicBehaviour*(*)()>* behaviourCreators;

		/**
		 * This map manages behaviours used by the currently running ALICA program.
		 */
		map<Behaviour*, BasicBehaviour*>* availableBehaviours;


		void loadTypesFromFile();
		void preLoadBehaviourThreads();
	};

} /* namespace alica */

#endif /* BEHAVIOURPOOL_H_ */
