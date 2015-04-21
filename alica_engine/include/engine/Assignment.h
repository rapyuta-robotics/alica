/*
 * Assignment.h
 *
 *  Created on: Jun 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef ASSIGNMENT_H_
#define ASSIGNMENT_H_



#include <vector>
#include <algorithm>
#include <memory>
#include <sstream>
#include "IAssignment.h"

using namespace std;
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

	/**
	 * Contains all allocation information for a single plan. This includes the robot-task mapping, robot-state mapping and success information.
	 */
	class Assignment : public IAssignment
	{
	public:
		Assignment(PartialAssignment* pa);
		Assignment(Plan* p,shared_ptr<AllocationAuthorityInfo> aai);
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
		//shared_ptr<vector<EntryPoint*> > getEntryPoints();
		short getEntryPointCount();
		shared_ptr<list<int> > getRobotsWorkingAndFinished(EntryPoint* ep);
		shared_ptr<list<int> > getUniqueRobotsWorkingAndFinished(EntryPoint* ep);
		shared_ptr<list<int> > getRobotsWorkingAndFinished(long epid);
		shared_ptr<SuccessCollection> getEpSuccessMapping();
		void setAllToInitialState(unique_ptr<list<int> > robots, EntryPoint* defep);
		bool removeRobot(int robotId);
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
		string toString();
		string toHackString();

	protected:
		/**
		 * The Plan this Assignment refers to
		 */
		Plan* plan;
		/**
		 * The robot-to-state mapping of this assignment.
		 */
		StateCollection* robotStateMapping;
		/**
		 * Information about succeeded tasks.
		 */
		shared_ptr<SuccessCollection> epSucMapping;
		AssignmentCollection* epRobotsMapping;
	};
} /* namespace alica */

#endif /* ASSIGNMENT_H_ */
