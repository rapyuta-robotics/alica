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

	vector<int>& StateCollection::getRobots()
	{
		return robots;
	}

	void StateCollection::setRobots(vector<int> robots)
	{
		this->robots = robots;
	}

	vector<State*>& StateCollection::getStates()
	{
		return states;
	}

	StateCollection::StateCollection(vector<int> robots, vector<State*> states)
	{
		this->robots = robots;
		this->states = states;
	}

	StateCollection::StateCollection(int maxSize) : robots(maxSize), states(maxSize)
	{
	}

	StateCollection::StateCollection(AssignmentCollection* ac)
	{
		for(int i = 0;i < ac->getSize(); i ++)
		{
			State* initialState = ac->getEp(i)->getState();
			for(auto r : *ac->getRobots(i))
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

	shared_ptr<vector<int> > StateCollection::getRobotsInStateSorted(State* s)
	{
		shared_ptr<vector<int> > ret= shared_ptr<vector<int> >();
		for (int i = 0; i < this->robots.size(); i++)
		{
			if (this->states[i] == s)
			{
				ret->push_back(this->robots[i]);
			}
		}
		sort(ret->begin(), ret->end());
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
				return;
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

	/**
	 * We are at new assignment, so everything is set to initial states, set them back:
	 */
	void StateCollection::reconsiderOldAssignment(shared_ptr<Assignment> oldOne, shared_ptr<Assignment> newOne)
	{
		if(oldOne->getPlan() != newOne->getPlan())
		{
			return;
		}
		//shared_ptr<vector<EntryPoint*> >eps = oldOne->getEntryPoints();
		EntryPoint* ep;
		for(short i = 0; i < oldOne->getEntryPointCount(); i++)
		{
			ep = oldOne->getEpRobotsMapping()->getEp(i);
			for(int rid : *(oldOne->getRobotsWorking(ep)))
			{
				auto iter = find(newOne->getRobotsWorking(ep)->begin(), newOne->getRobotsWorking(ep)->end(), rid);
				if(iter != newOne->getRobotsWorking(ep)->end())
				{
					this->setState(rid, oldOne->getRobotStateMapping()->getState(rid));
				}
			}
		}
	}


} /* namespace alica */
