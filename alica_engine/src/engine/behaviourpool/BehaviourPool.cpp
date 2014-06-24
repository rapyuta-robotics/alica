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
#include "engine/model/BehaviourConfiguration.h"
#include "engine/BasicBehaviour.h"

namespace alica
{

	BehaviourPool::BehaviourPool()
	{
		this->behaviourCreator = nullptr;
		this->availableBehaviours = new map<Behaviour*, shared_ptr<BasicBehaviour> >();
	}

	BehaviourPool::~BehaviourPool()
	{
		delete this->availableBehaviours;
	}

	bool BehaviourPool::init(IBehaviourCreator* bc)
	{
		if (this->behaviourCreator != nullptr)
		{
			delete this->behaviourCreator;
		}

		this->behaviourCreator = bc;

		auto behaviours = AlicaEngine::getInstance()->getPlanRepository()->getBehaviours();
		for (auto iter : behaviours)
		{
			shared_ptr<BasicBehaviour> basicBeh = this->behaviourCreator->createBehaviour(iter.second->getName());
			if (basicBeh != nullptr)
			{
				iter.second->setImplementation(basicBeh);
				this->availableBehaviours->insert(make_pair(iter.second, move(basicBeh)));
			}
			else
			{
				return false;
			}
		}
		return true;
	}

	void BehaviourPool::stopAll()
	{
		auto behaviours = AlicaEngine::getInstance()->getPlanRepository()->getBehaviours();
		for (auto iter : behaviours)
		{
			shared_ptr<BasicBehaviour> bbPtr = iter.second->getImplementation();
			if (bbPtr == nullptr)
			{
				cerr << "BP::stop(): Found Behaviour without an BasicBehaviour attached!" << endl;
				continue;
			}

			bbPtr->stop();

		}
	}

	void BehaviourPool::stopBehaviour(shared_ptr<RunningPlan> rp)
	{
		// TODO implement this
	}

	void BehaviourPool::startBehaviour(shared_ptr<RunningPlan> rp)
	{
		if (BehaviourConfiguration* bc = dynamic_cast<BehaviourConfiguration*>(rp->getPlan()))
		{
			shared_ptr<BasicBehaviour> bb = bc->getBehaviour()->getImplementation();
			if (bb != nullptr)
			{
				// set basic behaviours params and vars
				bb->setParameters(bc->getParameters());
				bb->setVariables(bc->getVariables());

				// set both directions rp <-> bb
				rp->setBasicBehaviour(bb);
				bb->setRunningPlan(rp);

				// start basic behaviour
				bb->setDueTime(bc->getDeferring());
				bb->setPeriod(1000 / bc->getFrequency());
				bb->start();
			}
		}
		else
		{
			cerr << "BP::stopBehaviour(): Cannot stop Behaviour of given RunningPlan! Plan Name: " << rp->getPlan()->getName() << " Plan Id: " << rp->getPlan()->getId() << endl;
		}
	}

} /* namespace alica */
