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

} /* namespace alica */
