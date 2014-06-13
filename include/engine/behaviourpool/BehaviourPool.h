/*
 * BehaviourPool.h
 *
 *  Created on: Jun 13, 2014
 *      Author: emmeda
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

	private:
		map<Behaviour*, type_index>* loadedBehaviours;

		map<Behaviour*, BasicBehaviour*>* usedBehaviours;

		template<typename T>
		BasicBehaviour* generateBasicBehaviourFromType (T type);

		void loadTypesFromFile ();
		void preLoadBehaviourThreads ();
	};

} /* namespace alica */

#include "engine/BasicBehaviour.h"

template<typename T>
inline alica::BasicBehaviour* alica::BehaviourPool::generateBasicBehaviourFromType(T type)
{
}

#endif /* BEHAVIOURPOOL_H_ */
