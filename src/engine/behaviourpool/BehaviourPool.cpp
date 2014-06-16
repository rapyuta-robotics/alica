/*
 * BehaviourPool.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: emmeda
 */
#define BP_DEBUG

#include "engine/behaviourpool/BehaviourPool.h"
#include "engine/RunningPlan.h"

namespace alica
{

	BehaviourPool::BehaviourPool()
	{
		this->behaviourCreators = new map<Behaviour*, createFunc> ();
		this->availableBehaviours = new map<Behaviour*, BasicBehaviour*>();
	}

	BehaviourPool::~BehaviourPool()
	{
	}

	void BehaviourPool::stop()
	{
	}

	void BehaviourPool::init()
	{
#ifdef BP_DEBUG
		cout << "BP: init() was called!" << endl;
#endif
		this->loadTypesFromFile();

	}

	bool BehaviourPool::isBehaviourAvailable(const Behaviour* b) const
	{
		return true;
	}

	void BehaviourPool::removeBehaviour(RunningPlan rp)
	{
	}

	void BehaviourPool::addBehaviour(RunningPlan rp)
	{
	}

	void BehaviourPool::loadTypesFromFile()
	{
#ifdef BP_DEBUG
		cout << "BP: loadTypesFromFile() was called!" << endl;
#endif
	}

	void BehaviourPool::preLoadBehaviourThreads()
	{
	}

	void BehaviourPool::registerBehaviour(Behaviour* behaviour, createFunc creatorFunction)
	{
		this->behaviourCreators->insert(make_pair(behaviour, creatorFunction));
	}

} /* namespace alica */
