/*
 * UtilityFunction.h
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef UTILITYFUNCTION_H_
#define UTILITYFUNCTION_H_

using namespace std;

#include <string>
#include <map>

namespace alica
{
	class Plan;
	struct TaskRoleStruct;
	class AlicaEngine;
	class IRoleAssignment;

	class UtilityFunction
	{
	public:
		UtilityFunction();
		virtual ~UtilityFunction();
		const double DIFFERENCETHRESHOLD = 0.0001;
	protected:
		Plan* plan;
		string name = "DefaultUtilityFunction";
		map<TaskRoleStruct*, double > taskRolePriorityMap;
		map<long, double> roleHighestPriorityMap;
		double priorityWeight;
		double similarityWeight;
		AlicaEngine* bpe;
		IRoleAssignment* ra;

	};

} /* namespace alica */

#endif /* UTILITYFUNCTION_H_ */
