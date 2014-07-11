/*
 * Assignment.h
 *
 *  Created on: Jun 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef ASSIGNMENT_H_
#define ASSIGNMENT_H_

using namespace std;


#include <vector>
#include <algorithm>
#include <memory>
#include "IAssignment.h"
#include <SystemConfig.h>

namespace alica
{

	class Plan;
	class StateCollection;
	class SuccessCollection;
	class AssignmentCollection;
	class EntryPoint;
	class PartialAssignment;
	class State;
	struct AllocationAuthorityInfo;

	class Assignment : public IAssignment
	{
	public:
		Assignment(PartialAssignment* pa);
		Assignment(Plan* p,AllocationAuthorityInfo* aai);
		Assignment(Plan* p);
		virtual ~Assignment();
		Plan* getPlan();
		void setPlan(Plan* plan);
		StateCollection* getRobotStateMapping();
		shared_ptr<vector<int> > getAllRobotsSorted();
		AssignmentCollection* getEpRobotsMapping();
		shared_ptr<vector<int> > getRobotsWorking(long epid);
		shared_ptr<vector<int> > getRobotsWorkingSorted(EntryPoint* ep);
		shared_ptr<vector<int> > getRobotsWorking(EntryPoint* ep);
		int totalRobotCount();
		vector<EntryPoint*> getEntryPoints();
		int getEntryPointCount();
		shared_ptr<list<int> > getRobotsWorkingAndFinished(EntryPoint* ep);
		shared_ptr<list<int> > getUniqueRobotsWorkingAndFinished(EntryPoint* ep);
		shared_ptr<list<int> > getRobotsWorkingAndFinished(long epid);
		SuccessCollection* getEpSuccessMapping();
		void setAllToInitialState(unique_ptr<list<int> > robots, EntryPoint* defep);
		void removeRobot(int robotId);
		void addRobot(int id, EntryPoint* e, State* s);
		bool isValid();
		bool isSuccessfull();
		bool isEqual(Assignment* otherAssignment);
		bool isEntryPointNonEmpty(EntryPoint* ep);
		bool updateRobot(int robot, EntryPoint* ep, State* s);
		bool updateRobot(int robot, EntryPoint* ep);
		bool removeRobot(int robot, EntryPoint* ep);
		string assignmentCollectionToString();
		void addRobot(int id, EntryPoint* e);
		void moveRobots(State* from, State* to);
		EntryPoint* entryPointOfRobot(int robot);
		shared_ptr<vector<int> >  getAllRobots();
		void clear();
		void setAllToInitialState(list<int> robots, EntryPoint* ep);
		string toString();
		string toHackString();

	protected:
		static bool allowIdling;
		Plan* plan;
		StateCollection* robotStateMapping;
		SuccessCollection* epSucMapping;
		AssignmentCollection* epRobotsMapping;
	};
} /* namespace alica */

#endif /* ASSIGNMENT_H_ */
