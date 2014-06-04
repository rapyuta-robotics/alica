/*
 * BehaviourConfiguration.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/BehaviourConfiguration.h"

namespace alica
{

	BehaviourConfiguration::BehaviourConfiguration()
	{
		this->eventDriven = false;
		this->frequency = 30;
		this->deferring = 0;
		this->parameters = (*new map<string,string>);

	}

	BehaviourConfiguration::BehaviourConfiguration(long id) : BehaviourConfiguration()
	{
		this->id = id;
	}

	BehaviourConfiguration::~BehaviourConfiguration()
	{

	}

	string BehaviourConfiguration::toString()
	{
		stringstream ss;
		ss << "#BehaviourConfiguration: " << this->getName()  << " " << this->getId() << endl;
		ss << "\tBehaviour: ";
		if(this->getBehaviour() != NULL) {
			ss << name << " " << this->getBehaviour()->getId();
		}
		ss << endl;"\tDeferring: " + this->getDeferring();

		//TODO finish to string
		return ss.str();
	}

//================== Getter and Setter =========================

	int BehaviourConfiguration::getDeferring() const
	{
		return deferring;
	}

	void BehaviourConfiguration::setDeferring(int deferring)
	{
		this->deferring = deferring;
	}

	bool BehaviourConfiguration::isEventDriven() const
	{
		return eventDriven;
	}

	void BehaviourConfiguration::setEventDriven(bool eventDriven)
	{
		this->eventDriven = eventDriven;
	}

	int BehaviourConfiguration::getFrequency() const
	{
		return frequency;
	}

	void BehaviourConfiguration::setFrequency(int frequency)
	{
		this->frequency = frequency;
	}

	const map<string, string>& BehaviourConfiguration::getParameters() const
	{
		return parameters;
	}

	void BehaviourConfiguration::setParameters(const map<string, string>& parameters)
	{
		this->parameters = parameters;
	}

	const Behaviour* BehaviourConfiguration::getBehaviour() const
	{
		return behaviour;
	}

	void BehaviourConfiguration::setBehaviour(const Behaviour* behaviour)
	{
		this->behaviour = behaviour;
	}

} /* namespace Alica */

