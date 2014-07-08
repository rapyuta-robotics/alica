/*
 * BasicBehaviour.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */
#define BEH_DEBUG

#include "engine/BasicBehaviour.h"
#include "engine/model/Behaviour.h"
#include "engine/AlicaEngine.h"
#include "engine/ITeamObserver.h"
#include "engine/RunningPlan.h"
#include "engine/model/BehaviourConfiguration.h"
#include <Timer.h>

namespace alica
{

	BasicBehaviour::BasicBehaviour(string name)
	{
		this->name = name;
		this->parameters = nullptr;
		this->timer = new supplementary::Timer(0, 0, false);
		this->failure = false;
		this->success = false;
		this->callInit = true;
		this->running = false;
		this->started = false;
		this->runThread = new thread(&BasicBehaviour::runInternal, this);
	}

	BasicBehaviour::~BasicBehaviour()
	{
	}

	const string BasicBehaviour::getName() const
	{
		return this->name;
	}

	void BasicBehaviour::setName(string name)
	{
		this->name = name;
	}

	shared_ptr<map<string, string>> BasicBehaviour::getParameters()
	{
		return this->parameters;
	}

	void BasicBehaviour::setParameters(shared_ptr<map<string, string>> parameters)
	{
		this->parameters = parameters;
	}

	shared_ptr<list<Variable*> > BasicBehaviour::getVariables()
	{
		return this->variables;
	}

	void BasicBehaviour::setVariables(shared_ptr<list<Variable*> > variables)
	{
		this->variables = variables;
	}

	int BasicBehaviour::getDelayedStart() const
	{
		return this->timer->getDelayedStart();
	}

	void BasicBehaviour::setDelayedStart(long msDelayedStart)
	{
		this->timer->setDelayedStart(msDelayedStart);
	}

	int BasicBehaviour::getInterval() const
	{
		return this->timer->getInterval();
	}

	void BasicBehaviour::setInterval(long msInterval)
	{
		this->timer->setInterval(msInterval);
	}

	/**
	 * Convenience method to obtain the robot's own id.
	 * @return the own robot id
	 */
	int BasicBehaviour::getOwnId()
	{
		return AlicaEngine::getInstance()->getTeamObserver()->getOwnId();
	}
	/**
	 * Starts the execution of this Behaviour (either Timer, or Event triggered)
	 */
	void BasicBehaviour::start()
	{
		// TODO implement this
		this->started = true;
		this->callInit = true;
	}

	/**
	 * Stops the execution of this Behaviour
	 */
	void BasicBehaviour::stop()
	{
		// TODO implement this
		this->started = false;
	}

	bool BasicBehaviour::pause()
	{
		return this->timer->pause();
	}

	bool BasicBehaviour::restart()
	{
		return this->timer->restart();
	}

	const shared_ptr<RunningPlan>& BasicBehaviour::getRunningPlan() const
	{
		return runningPlan;
	}

	void BasicBehaviour::setRunningPlan(const shared_ptr<RunningPlan>& runningPlan)
	{
		this->runningPlan = runningPlan;
	}

	void BasicBehaviour::initInternal()
	{
		this->success = false;
		this->failure = false;
		this->callInit = false;
		try
		{
			this->initialiseParameters();
		}
		catch (exception& e)
		{
			cerr << "BB: Exception in Behaviour-INIT of: " << this->getName() << endl << e.what() << endl;
		}
	}

	void BasicBehaviour::runInternal()
	{
		sleep(3);
		unique_lock<mutex> lck(runCV_mtx);
		while (!AlicaEngine::getInstance()->isTerminating() && this->started)
		{
			this->runCV.wait(lck, [&]
			{return this->timer->isRunning();}); // protection against spurious wake-ups
			if (!this->started)
				return;
			if (this->callInit)
				this->initialiseParameters();
#ifdef BEH_DEBUG
			chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
#endif
			// TODO: pass something like an eventarg (to be implemented) class-member, which could be set for an event triggered (to be implemented) behaviour.
			this->run(nullptr);
#ifdef BEH_DEBUG
			BehaviourConfiguration* conf = dynamic_cast<BehaviourConfiguration*>(this->getRunningPlan()->getPlan());
			if (conf->isEventDriven())
			{
				double dura = (std::chrono::high_resolution_clock::now() - start).count() / 1000000.0
						- 1.0 / conf->getFrequency() * 1000.0;
				if (dura > 0.1)
				{ //Behaviour "+conf.Behaviour.Name+" exceeded runtime by {0,1:0.##}ms!",delta
					cout << "BB: Behaviour " << conf->getBehaviour()->getName() << " exceeded runtime by \t" << dura
							<< "ms!" << endl;
				}
			}
#endif
		}
	}

} /* namespace alica */

