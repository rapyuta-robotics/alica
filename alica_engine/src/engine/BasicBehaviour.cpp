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
#include "engine/model/Variable.h"
#include "engine/model/Plan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Task.h"
#include <Timer.h>

#include <iostream>

namespace alica
{
	/**
	 * Basic constructor. Initialises the timer. Should only be called from the constructor of inheriting classes.
	 * If using eventTrigger set behaviourTrigger and register runCV
	 * @param name The name of the behaviour
	 */
	BasicBehaviour::BasicBehaviour(string name) :
			name(name), parameters(nullptr), failure(false), success(false), callInit(true), started(true), runCV()
	{
		this->running = false;
		this->timer = new supplementary::Timer(0, 0);
		this->timer->registerCV(&this->runCV);
		this->behaviourTrigger = nullptr;
		this->runThread = new thread(&BasicBehaviour::runInternal, this);
	}

	BasicBehaviour::~BasicBehaviour()
	{
		this->started = false;
		this->runCV.notify_all();
		this->timer->start();
		this->runThread->join();
		delete this->runThread;
		delete this->timer;
		if (behaviourTrigger != nullptr)
		{
			delete this->behaviourTrigger;
		}
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

	Variable* BasicBehaviour::getVariablesByName(string name)
	{
		list<Variable*>::iterator it;
		for (it = variables->begin(); it != variables->end(); it++)
		{
			Variable* v = *it;
			if (v->getName() == name)
			{
				return v;
			}
		}
		return nullptr;
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
		return this->runningPlan->getOwnID();
	}

	/**
	 * Stops the execution of this BasicBehaviour.
	 */
	bool BasicBehaviour::stop()
	{
		this->success = false;
		this->failure = false;
		if (behaviourTrigger == nullptr)
		{
			return this->timer->stop();
		}
		else
		{
			this->running = false;
		}
		return true;
	}

	/**
	 * Starts the execution of this BasicBehaviour.
	 */
	bool BasicBehaviour::start()
	{
		this->callInit = true;
		if (behaviourTrigger == nullptr)
		{
			return this->timer->start();
		}
		else
		{
			this->running = true;
		}
		return true;
	}

	shared_ptr<RunningPlan> BasicBehaviour::getRunningPlan()
	{
		return runningPlan;
	}

	void BasicBehaviour::setRunningPlan(shared_ptr<RunningPlan> runningPlan)
	{
		this->runningPlan = runningPlan;
	}

	bool BasicBehaviour::isSuccess() const
	{
		return success;
	}

	bool BasicBehaviour::isFailure() const
	{
		return failure;
	}

	void BasicBehaviour::setTrigger(supplementary::ITrigger* trigger)
	{
		this->behaviourTrigger = trigger;
		this->behaviourTrigger->registerCV(&this->runCV);
	}

	shared_ptr<vector<int>> BasicBehaviour::robotsInEntryPointOfHigherPlan(EntryPoint* ep)
	{
		if (ep == nullptr)
		{
			return nullptr;
		}
		shared_ptr<RunningPlan> cur = this->runningPlan->getParent().lock();
		while (cur != nullptr)
		{
			if (((Plan*)cur->getPlan())->getEntryPoints().find(ep->getId()) != ((Plan*)cur->getPlan())->getEntryPoints().end())
			{
				return cur->getAssignment()->getRobotsWorking(ep);
			}
			cur = cur->getParent().lock();
		}
		return nullptr;
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

	bool BasicBehaviour::getParameter(string key, string& valueOut)
	{
		for (auto pair : *parameters)
		{
			if (pair.first == key)
			{
				valueOut = pair.second;
				return true;
			}
		}
		valueOut = "";
		return false;
	}

	void BasicBehaviour::runInternal()
	{
		unique_lock<mutex> lck(runCV_mtx);
		while (this->started)
		{
			this->runCV.wait(lck, [&]
			{
				if(behaviourTrigger == nullptr)
				{
					return !this->started || this->timer->isNotifyCalled(&runCV);
				}
				else
				{
					return !this->started || (this->behaviourTrigger->isNotifyCalled(&runCV) && this->running);
				}
			}); // protection against spurious wake-ups
			if (!this->started)
				return;

			if (this->callInit)
				this->initInternal();
#ifdef BEH_DEBUG
			chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
#endif
			// TODO: pass something like an eventarg (to be implemented) class-member, which could be set for an event triggered (to be implemented) behaviour.
			try
			{
				if (behaviourTrigger == nullptr)
				{
					this->run((void*)timer);
				}
				else
				{
					this->run((void*)behaviourTrigger);
				}
			}
			catch (exception& e)
			{
				cerr << "Exception catched: " << e.what() << endl;
			}
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
			if (behaviourTrigger == nullptr)
			{
				this->timer->setNotifyCalled(false, &runCV);
			}
			else
			{
				this->behaviourTrigger->setNotifyCalled(false, &runCV);
			}
		}
	}

	EntryPoint* BasicBehaviour::getParentEntryPoint(string taskName)
	{
		shared_ptr<RunningPlan> parent = this->runningPlan->getParent().lock();
		if (parent == nullptr)
		{
			return nullptr;
		}
		for (pair<long, EntryPoint*> e : ((Plan*)parent->getPlan())->getEntryPoints())
		{
			if (e.second->getTask()->getName() == taskName)
			{
				return e.second;
			}
		}
		return nullptr;
	}

	EntryPoint* BasicBehaviour::getHigherEntryPoint(string planName, string taskName)
	{
		shared_ptr<RunningPlan> cur = this->runningPlan->getParent().lock();
		while (cur != nullptr)
		{
			if (cur->getPlan()->getName() == planName)
			{
				for (pair<long, EntryPoint*> e : ((Plan*)cur->getPlan())->getEntryPoints())
				{
					if (e.second->getTask()->getName() == taskName)
					{
						return e.second;
					}
				}
				return nullptr;
			}
			cur = cur->getParent().lock();
		}
		return nullptr;
	}

} /* namespace alica */

