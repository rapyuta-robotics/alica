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

namespace alica
{

	class Plan;
	class StateCollection;
	class AssignmentCollection;
	class EntryPoint;

	class Assignment : public virtual IAssignment
	{
	public:
		Assignment();
		virtual ~Assignment();
		Plan* getPlan();
		void setPlan(Plan* plan);
		StateCollection* getRobotStateMapping();
		shared_ptr<vector<int> > getAllRobotsSorted();
		AssignmentCollection* getEpRobotsMapping();
		shared_ptr<vector<int> > getRobotsWorking(long epid);
		shared_ptr<vector<int> > getRobotsWorkingSorted(EntryPoint* ep);
		shared_ptr<vector<int> > getRobotsWorking(EntryPoint* ep);

	protected:
		Plan* plan;
		StateCollection* robotStateMapping;
		AssignmentCollection* epRobotsMapping;
	};

} /* namespace alica */

#endif /* ASSIGNMENT_H_ */
