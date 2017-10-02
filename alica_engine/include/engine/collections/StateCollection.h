/*
 * StateCollection.h
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#ifndef STATECOLLECTION_H_
#define STATECOLLECTION_H_


#include "supplementary/IAgentID.h"
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
		StateCollection(vector<const supplementary::IAgentID*> robotIds, vector<State*> states);
		StateCollection(int maxSize);
		StateCollection(AssignmentCollection* ac);
		virtual ~StateCollection();
		vector<const supplementary::IAgentID*>& getRobots();
		void setRobots(vector<const supplementary::IAgentID*> robotIds);
		vector<State*>& getStates();
		void setStates(vector<State*> states);
		int getCount();
		State* getState(const supplementary::IAgentID* robotId);
		unordered_set<const supplementary::IAgentID*> getRobotsInState(State* s);
		shared_ptr<vector<const supplementary::IAgentID*> > getRobotsInStateSorted(State* s);
		unordered_set<const supplementary::IAgentID*> getRobotsInState(long sid);
		void removeRobot(const supplementary::IAgentID* robotId);
		void clear();
		State* stateOfRobot(const supplementary::IAgentID* robotId);
		void setState(const supplementary::IAgentID* robotId, State* state);
		void setStates(vector<const supplementary::IAgentID*> robotIds, State* state);
		string toString();
		void setInitialState(const supplementary::IAgentID* robotId, EntryPoint* ep);
		void reconsiderOldAssignment(shared_ptr<Assignment> oldOne, shared_ptr<Assignment> newOne);

	protected:
		vector<const supplementary::IAgentID*> robotIds;
		vector<State*> states;
	};

} /* namespace alica */

#endif /* STATECOLLECTION_H_ */
