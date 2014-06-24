/*
 * BasicBehaviour.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#include "engine/BasicBehaviour.h"
#include "engine/AlicaEngine.h"
#include "engine/ITeamObserver.h"

namespace alica
{

	BasicBehaviour::BasicBehaviour(string name)
	{
		this->name = name;
		this->parameters = nullptr;
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

	shared_ptr<map<string,string>> BasicBehaviour::getParameters()
	{
		return this->parameters;
	}

	void BasicBehaviour::setParameters(shared_ptr<map<string,string>> parameters)
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

	int BasicBehaviour::getDueTime() const
	{
		return dueTime;
	}

	void BasicBehaviour::setDueTime(int dueTime)
	{
		this->dueTime = dueTime;
	}

	int BasicBehaviour::getPeriod() const
	{
		return period;
	}

	void BasicBehaviour::setPeriod(int period)
	{
		this->period = period;
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
	}

	/**
	 * Stops the execution of this Behaviour
	 */
	void BasicBehaviour::stop()
	{
		// TODO implement this
	}

	const shared_ptr<RunningPlan>& BasicBehaviour::getRunningPlan() const
	{
		return runningPlan;
	}

	void BasicBehaviour::setRunningPlan(const shared_ptr<RunningPlan>& runningPlan)
	{
		this->runningPlan = runningPlan;
	}


} /* namespace alica */


