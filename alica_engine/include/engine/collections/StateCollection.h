/*
 * StateCollection.h
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#ifndef STATECOLLECTION_H_
#define STATECOLLECTION_H_


#include "engine/IRobotID.h"
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <sstream>
#include <memory>
#include <iostream>

using namespace std;
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
		StateCollection(vector<alica::IRobotID> robotIds, vector<State*> states);
		StateCollection(int maxSize);
		StateCollection(AssignmentCollection* ac);
		virtual ~StateCollection();
		vector<alica::IRobotID>& getRobots();
		void setRobots(vector<alica::IRobotID> robotIds);
		vector<State*>& getStates();
		void setStates(vector<State*> states);
		int getCount();
		State* getState(alica::IRobotID robotId);
		unordered_set<alica::IRobotID> getRobotsInState(State* s);
		shared_ptr<vector<alica::IRobotID> > getRobotsInStateSorted(State* s);
		unordered_set<alica::IRobotID> getRobotsInState(long sid);
		void removeRobot(alica::IRobotID robotId);
		void clear();
		State* stateOfRobot(alica::IRobotID robotId);
		void setState(alica::IRobotID robotId, State* state);
		void setStates(vector<alica::IRobotID> robotIds, State* state);
		string toString();
		void setInitialState(alica::IRobotID robotId, EntryPoint* ep);
		void reconsiderOldAssignment(shared_ptr<Assignment> oldOne, shared_ptr<Assignment> newOne);

	protected:
		vector<alica::IRobotID> robotIds;
		vector<State*> states;
	};

} /* namespace alica */

#endif /* STATECOLLECTION_H_ */
