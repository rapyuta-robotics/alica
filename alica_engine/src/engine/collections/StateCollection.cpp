/*
 * StateCollection.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#include "engine/collections/StateCollection.h"

namespace alica
{

	StateCollection::StateCollection()
	{

	}

	StateCollection::~StateCollection()
	{
	}

	list<int> StateCollection::getRobots()
	{
		return robots;
	}

	void StateCollection::setRobots(list<int> robots)
	{
		this->robots = robots;
	}

	list<State*> StateCollection::getStates()
	{
		return states;
	}

	StateCollection::StateCollection(list<int> robots, list<State*> states)
	{
		this->robots = robots;
		this->states = states;
	}

	StateCollection::StateCollection(int maxSize)
	{
		this->robots = list<int>(maxSize);
		this->states = list<State*>(maxSize);
	}

	StateCollection::StateCollection(AssignmentCollection* ac)
	{
		//TODO AssignemntCollection
	}

	void StateCollection::setStates(list<State*> states)
	{
		this->states = states;
	}

	int StateCollection::getCount()
	{
		return this->robots.size();
	}

	State* StateCollection::getState(int r)
	{
		for(int i = 0; i < this->robots.size(); i++)
		{
			//TODO
//			if(this->robots == r)
//			{
//				return this->states[i];
//			}
		}
		return nullptr;
	}

	unordered_set<int> StateCollection::getRobotsInState(State* s)
	{
		unordered_set<int> ret;

		return ret;
	}

	list<int> StateCollection::getRobotsInStateSorted(State* s)
	{
	}

	unordered_set<int> StateCollection::getRobotsInState(long sid)
	{
	}

	void StateCollection::removeRobot(int r)
	{
	}

	void StateCollection::clear()
	{
	}

	State* StateCollection::stateOfRobot(int robot)
	{
	}

	void StateCollection::setState(int robot, State* state)
	{
	}

	void StateCollection::toString()
	{
	}

	void StateCollection::setInitialState(int robot, EntryPoint* ep)
	{
	}

	void StateCollection::reconsiderOldAssignment(Assignment* old, Assignment* newOne)
	{
	}

} /* namespace alica */
