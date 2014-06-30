/*
 * StateCollection.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#include "engine/collections/StateCollection.h"
#include "engine/model/State.h"
#include "engine/model/EntryPoint.h"
#include "engine/collections/AssignmentCollection.h"
#include "engine/Assignment.h"

namespace alica
{

	StateCollection::StateCollection()
	{

	}

	StateCollection::~StateCollection()
	{
	}

	vector<int> StateCollection::getRobots()
	{
		return robots;
	}

	void StateCollection::setRobots(vector<int> robots)
	{
		this->robots = robots;
	}

	vector<State*> StateCollection::getStates()
	{
		return states;
	}

	StateCollection::StateCollection(vector<int> robots, vector<State*> states)
	{
		this->robots = robots;
		this->states = states;
	}

	StateCollection::StateCollection(int maxSize)
	{
		this->robots = vector<int>(maxSize);
		this->states = vector<State*>(maxSize);
	}

	StateCollection::StateCollection(AssignmentCollection* ac)
	{
		for(int i = 0;i < ac->getCount(); i ++)
		{
			State* initialState = ac->getEntryPoints()[i]->getState();
			for(int r : *(ac->getRobots()[i]))
			{
				this->setState(r,initialState);
			}
		}
	}

	void StateCollection::setStates(vector<State*> states)
	{
		this->states = states;
	}

	int StateCollection::getCount()
	{
		return this->robots.size();
	}

	State* StateCollection::getState(int r)
	{
		for (int i = 0; i < this->robots.size(); i++)
		{
			if (this->robots[i] == r)
			{
				return this->states[i];
			}
		}
		return nullptr;
	}

	unordered_set<int> StateCollection::getRobotsInState(State* s)
	{
		unordered_set<int> ret;
		for (int i = 0; i < this->robots.size(); i++)
		{
			if (this->states[i] == s)
			{
				ret.insert(this->robots[i]);
			}
		}

		return ret;
	}

	vector<int> StateCollection::getRobotsInStateSorted(State* s)
	{
		vector<int> ret;
		for (int i = 0; i < this->robots.size(); i++)
		{
			if (this->states[i] == s)
			{
				ret.push_back(this->robots[i]);
			}
		}
		sort(ret.begin(), ret.end());
		return ret;
	}

	unordered_set<int> StateCollection::getRobotsInState(long sid)
	{
		unordered_set<int> ret;
		for (int i = 0; i < this->robots.size(); i++)
		{
			if (this->states[i]->getId() == sid)
			{
				ret.insert(this->robots[i]);
			}
		}

		return ret;
	}

	void StateCollection::removeRobot(int r)
	{
		for(int i = 0; i < this->states.size();i++)
		{
			if(this->robots[i] == r)
			{
				this->robots.erase(robots.begin() + i);
				this->states.erase(states.begin() + i);
				return;
			}
		}
	}

	void StateCollection::clear()
	{
		this->robots.clear();
		this->states.clear();
	}

	State* StateCollection::stateOfRobot(int robot)
	{
		for (int i = 0; i < this->robots.size(); i++)
		{
			if (this->robots[i] == robot)
			{
				return this->states[i];
			}
		}
		return nullptr;
	}

	void StateCollection::setState(int robot, State* state)
	{
		for (int i = 0; i < this->robots.size(); i++)
		{
			if (this->robots[i] == robot)
			{
				this->states[i] = state;
			}
		}
		this->robots.push_back(robot);
		this->states.push_back(state);

	}

	string StateCollection::toString()
	{
		stringstream ss;
		for(int i = 0; i < robots.size();i++)
		{
			ss << "R: " << this->robots[i] << " in State: ";
			if(this->states[i] == nullptr)
			{
				ss << "NULL" << endl;
			}
			else
			{
				ss << this->states[i]->getName() << " (" << this->states[i]->getId()<< ") " << endl;
			}
		}
		return ss.str();
	}

	void StateCollection::setInitialState(int robot, EntryPoint* ep)
	{
		setState(robot, ep->getState());
	}

	void StateCollection::setStates(vector<int> robots, State* state)
	{
		for(int i = 0; i <  robots.size(); i++)
		{
			setState(robots[i], state);
		}
	}

	void StateCollection::reconsiderOldAssignment(Assignment* oldOne, Assignment* newOne)
	{
		if(oldOne->getPlan() != newOne->getPlan())
		{
			return;
		}
		vector<EntryPoint*> eps = oldOne->getEntyPoints();
		for(int i = 0; i < eps.size(); i++)
		{
			for(int rid : oldOne->getRobotsWorking(eps[i]))
			{
				auto iter = find(newOne->getRobotsWorking(eps[i]).begin(), newOne->getRobotsWorking(eps[i]).end(), rid);
				if(iter != newOne->getRobotsWorking(eps[i]).end())
				{
					this->setState(rid, oldOne->getRobotStateMapping()->getState(rid));
				}
			}
		}
	}


} /* namespace alica */
