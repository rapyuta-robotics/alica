/*
 * PartialAssignment.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef PARTIALASSIGNMENT_H_
#define PARTIALASSIGNMENT_H_

using namespace std;

#include <vector>
#include <list>
#include <limits>

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

	class PartialAssignment
	{
	public:
		virtual ~PartialAssignment();
		static void init();
		void clear();
		static void reset();
		static PartialAssignment* getNew(vector<int> robots, Plan* plan, SuccessCollection* sucCol);
		static PartialAssignment* getNew(PartialAssignment* oldPA);



	private:
		const int INFINIT = numeric_limits<int>::max();
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
		list<int> unAssignedRobots;
		Plan* plan;
		const long PRECISION = 1073741824;
		long compareVal = 0;

		//TODO c# line 203
	};

} /* namespace alica */

#endif /* PARTIALASSIGNMENT_H_ */
