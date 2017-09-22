/*
 * Assignment.h
 *
 *  Created on: Jun 27, 2014
 *      Author: Stefan Jakob
 */

#ifndef ASSIGNMENT_H_
#define ASSIGNMENT_H_


#include "IAssignment.h"
#include "IRobotID.h"
#include <vector>
#include <memory>
#include <algorithm>
#include <sstream>

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
		shared_ptr<vector<const alica::IRobotID*> > getAllRobotsSorted();
		AssignmentCollection* getEpRobotsMapping();
		shared_ptr<vector<const alica::IRobotID*> > getRobotsWorking(long epid);
		shared_ptr<vector<const alica::IRobotID*> > getRobotsWorkingSorted(EntryPoint* ep);
		shared_ptr<vector<const alica::IRobotID*> > getRobotsWorking(EntryPoint* ep);
		int totalRobotCount();
		//shared_ptr<vector<EntryPoint*> > getEntryPoints();
		short getEntryPointCount();
		shared_ptr<list<const alica::IRobotID*> > getRobotsWorkingAndFinished(EntryPoint* ep);
		shared_ptr<list<const alica::IRobotID*> > getUniqueRobotsWorkingAndFinished(EntryPoint* ep);
		shared_ptr<list<const alica::IRobotID*> > getRobotsWorkingAndFinished(long epid);
		shared_ptr<SuccessCollection> getEpSuccessMapping();
		void setAllToInitialState(unique_ptr<list<const alica::IRobotID*> > robotIds, EntryPoint* defep);
		bool removeRobot(const alica::IRobotID* robotId);
		void addRobot(const alica::IRobotID* robotId, EntryPoint* e, State* s);
		bool isValid();
		bool isSuccessfull();
		bool isEqual(Assignment* otherAssignment);
		bool isEntryPointNonEmpty(EntryPoint* ep);
		bool updateRobot(const alica::IRobotID* robotId, EntryPoint* ep, State* s);
		bool updateRobot(const alica::IRobotID* robotId, EntryPoint* ep);
		bool removeRobot(const alica::IRobotID* robotId, EntryPoint* ep);
		string assignmentCollectionToString();
		void addRobot(const alica::IRobotID* robotId, EntryPoint* e);
		void moveRobots(State* from, State* to);
		EntryPoint* getEntryPointOfRobot(const alica::IRobotID* robotId);
		shared_ptr<vector<const alica::IRobotID*> >  getAllRobots();
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
