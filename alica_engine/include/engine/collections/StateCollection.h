/*
 * StateCollection.h
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#ifndef STATECOLLECTION_H_
#define STATECOLLECTION_H_

using namespace std;

#include <list>
#include <unordered_set>

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
		StateCollection(list<int> robots, list<State*> states);
		StateCollection(int maxSize);
		StateCollection(AssignmentCollection* ac);
		virtual ~StateCollection();
		list<int> getRobots();
		void setRobots(list<int> robots);
		list<State*> getStates();
		void setStates(list<State*> states);
		int getCount();
		State* getState(int r);
		unordered_set<int> getRobotsInState(State* s);
		list<int> getRobotsInStateSorted(State* s);
		unordered_set<int> getRobotsInState(long sid);
		void removeRobot(int r);
		void clear();
		State* stateOfRobot(int robot);
		void setState(int robot, State* state);
		void toString();
		void setInitialState(int robot, EntryPoint* ep);
		void reconsiderOldAssignment(Assignment* old, Assignment* newOne);

	protected:
		list<int> robots;
		list<State*> states;
	};

} /* namespace alica */

#endif /* STATECOLLECTION_H_ */
