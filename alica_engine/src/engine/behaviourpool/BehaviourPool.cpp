/*
 * BehaviourPool.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: emmeda
 */
#define BP_DEBUG

#include "engine/behaviourpool/BehaviourPool.h"
#include "engine/RunningPlan.h"
#include "engine/IBehaviourCreator.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/Behaviour.h"
#include "engine/BasicBehaviour.h"

namespace alica
{

	BehaviourPool::BehaviourPool()
	{
		this->behaviourCreator = nullptr;
		this->availableBehaviours = new map<Behaviour*, unique_ptr<BasicBehaviour> >();
	}

	BehaviourPool::~BehaviourPool()
	{
		delete this->availableBehaviours;
	}

	void BehaviourPool::stop()
	{
	}

	bool BehaviourPool::init(IBehaviourCreator* bc)
	{
		if (this->behaviourCreator != nullptr)
		{
			delete this->behaviourCreator;
		}

		this->behaviourCreator = bc;

		std::map<long int, alica::Behaviour*> behaviours =
				AlicaEngine::getInstance()->getPlanRepository()->getBehaviours();
		for (auto iter : behaviours)
		{
			unique_ptr<BasicBehaviour> basicBeh = this->behaviourCreator->createBehaviour(iter.second->getName());
			if (basicBeh != nullptr)
			{
				this->availableBehaviours->insert(make_pair(iter.second, move(basicBeh)));
			} else
			{
				return false;
			}
		}
		return true;
	}

	bool BehaviourPool::isBehaviourAvailable(Behaviour* b) const
	{
		try
		{
			this->availableBehaviours->at(b);
			return true;
		}
		catch (const std::out_of_range& oor)
		{
			cerr << "Out of Range error: " << oor.what() << '\n';
			return false;
		}
	}

	void BehaviourPool::removeBehaviour(RunningPlan rp)
	{
	}

	void BehaviourPool::addBehaviour(RunningPlan rp)
	{
	}

} /* namespace alica */
