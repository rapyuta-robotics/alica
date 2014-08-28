/*
 * StateCollection.h
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#ifndef STATECOLLECTION_H_
#define STATECOLLECTION_H_

using namespace std;

#include <vector>
#include <unordered_set>
#include <algorithm>
#include <sstream>
#include <memory>
#include <iostream>

namespace alica
{

	class State;
	class AssignmentCollection;
	class EntryPoint;
	class Assignment;

	class StateCollection
	{
	public:
		StateCollection();
		StateCollection(vector<int> robots, vector<State*> states);
		StateCollection(int maxSize);
		StateCollection(AssignmentCollection* ac);
		virtual ~StateCollection();
		vector<int>& getRobots();
		void setRobots(vector<int> robots);
		vector<State*>& getStates();
		void setStates(vector<State*> states);
		int getCount();
		State* getState(int r);
		unordered_set<int>& getRobotsInState(State* s);
		shared_ptr<vector<int> > getRobotsInStateSorted(State* s);
		unordered_set<int>& getRobotsInState(long sid);
		void removeRobot(int r);
		void clear();
		State* stateOfRobot(int robot);
		void setState(int robot, State* state);
		void setStates(vector<int> robots, State* state);
		string toString();
		void setInitialState(int robot, EntryPoint* ep);
		void reconsiderOldAssignment(Assignment* oldOne, Assignment* newOne);

	protected:
		vector<int> robots;
		vector<State*> states;
	};

} /* namespace alica */

#endif /* STATECOLLECTION_H_ */
