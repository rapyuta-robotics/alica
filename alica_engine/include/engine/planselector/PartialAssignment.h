/*
 * PartialAssignment.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef PARTIALASSIGNMENT_H_
#define PARTIALASSIGNMENT_H_

#define PADEBUG
#define SUCDEBUG

using namespace std;

#include <vector>
#include <list>
#include <limits>
#include <sstream>
#include <string>
#include <algorithm>
#include <memory>

#include <SystemConfig.h>


namespace alica
{

	class EpByTaskComparer;
	class EntryPoint;
	class Plan;
	class SuccessCollection;
	class UtilityFunction;
	class AssignmentCollection;
	class DynCardinality;
	class SimplePlanTree;

	class PartialAssignment
	{
	public:
		virtual ~PartialAssignment();
		static void init();
		void clear();
		static void reset();
		static PartialAssignment* getNew(vector<int> robots, Plan* plan, SuccessCollection* sucCol);
		static PartialAssignment* getNew(PartialAssignment* oldPA);
		int getEntryPointCount();
		int totalRobotCount();
		shared_ptr<vector<int> > getRobotsWorking(EntryPoint* ep);
		shared_ptr<vector<int> > getRobotsWorking(long epid);
		shared_ptr<list<int> > getRobotsWorkingAndFinished(EntryPoint* ep);
		shared_ptr<list<int> > getRobotsWorkingAndFinished(long epid);
		shared_ptr<list<int> > getUniqueRobotsWorkingAndFinished(EntryPoint* ep);
		bool addIfAlreadyAssigned(SimplePlanTree spt, int robot);
		bool assignRobot(int robot, int index);
		shared_ptr<list<PartialAssignment*> > expand();
		bool isValid();
		bool isGoal();
		bool compareTo(PartialAssignment* thisPa, PartialAssignment* newPa);
		int getHashCode();
		string toString();
		AssignmentCollection* getEpRobotsMapping();
		Plan* getPlan();
		UtilityFunction* getUtilFunc();
		SuccessCollection* getEpSuccessMapping();
		int numUnAssignedRobots();
		vector<int> getUnAssignedRobots();
		double getMax();
		void setMax(double max);
		double getMin();
		void setMin(double min);
		string assignmentCollectionToString();
		vector<EntryPoint*> getEntryPoints();

	private:
		const int INFINIT = numeric_limits<int>::max();
		static int pow(int x, int y);
		PartialAssignment();

	protected:
		static int maxCount;
		static int maxEpsCount;
		static int curIndex;
		static vector<PartialAssignment*> daPAs;
		static EpByTaskComparer epByTaskComparer;
		static bool allowIdling;
		static EntryPoint* idleEP;
		UtilityFunction* utilFunc;
		double min;
		double max;
		AssignmentCollection* epRobotsMapping;
		vector<int> robots;
		vector<DynCardinality*> dynCardinalities;
		vector<int> unAssignedRobots;
		Plan* plan;
		const long PRECISION = 1073741824;
		long compareVal = 0;
		bool hashCalculated;
		int hash = 0;
		SuccessCollection* epSuccessMapping;


	};

} /* namespace alica */

#endif /* PARTIALASSIGNMENT_H_ */
