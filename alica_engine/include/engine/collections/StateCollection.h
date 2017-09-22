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
		StateCollection(vector<const alica::IRobotID*> robotIds, vector<State*> states);
		StateCollection(int maxSize);
		StateCollection(AssignmentCollection* ac);
		virtual ~StateCollection();
		vector<const alica::IRobotID*>& getRobots();
		void setRobots(vector<const alica::IRobotID*> robotIds);
		vector<State*>& getStates();
		void setStates(vector<State*> states);
		int getCount();
		State* getState(const alica::IRobotID* robotId);
		unordered_set<const alica::IRobotID*> getRobotsInState(State* s);
		shared_ptr<vector<const alica::IRobotID*> > getRobotsInStateSorted(State* s);
		unordered_set<const alica::IRobotID*> getRobotsInState(long sid);
		void removeRobot(const alica::IRobotID* robotId);
		void clear();
		State* stateOfRobot(const alica::IRobotID* robotId);
		void setState(const alica::IRobotID* robotId, State* state);
		void setStates(vector<const alica::IRobotID*> robotIds, State* state);
		string toString();
		void setInitialState(const alica::IRobotID* robotId, EntryPoint* ep);
		void reconsiderOldAssignment(shared_ptr<Assignment> oldOne, shared_ptr<Assignment> newOne);

	protected:
		vector<const alica::IRobotID*> robotIds;
		vector<State*> states;
	};

} /* namespace alica */

#endif /* STATECOLLECTION_H_ */
