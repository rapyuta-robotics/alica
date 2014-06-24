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
		this->parameters = new map<string, string>();
	}

	BasicBehaviour::BasicBehaviour()
	{
		this->name = "Test1234";
		this->parameters = new map<string, string>();
	}

	BasicBehaviour::~BasicBehaviour()
	{
		delete this->parameters;
	}

	const string BasicBehaviour::getName() const
	{
		return this->name;
	}

	void BasicBehaviour::setName(string name)
	{
		this->name = name;
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


} /* namespace alica */


