/*
 * UtilityFunction.h
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef UTILITYFUNCTION_H_
#define UTILITYFUNCTION_H_
#define UFDEBUG

using namespace std;

#include <string>
#include <map>
#include <list>
#include <algorithm>
#include <vector>
#include <sstream>

namespace alica
{
	class Plan;
	class AlicaEngine;
	class IRoleAssignment;
	class USummand;
	class RunningPlan;
	class IAssignment;
	struct UtilityInterval;
	struct TaskRoleStruct;

	class UtilityFunction
	{
	public:
		UtilityFunction(string name, list<USummand*> utilSummands, double priorityWeight, double similarityWeight, Plan* plan);
		virtual ~UtilityFunction();
		list<USummand*> getUtilSummands();
		void setUtilSummands(list<USummand*> utilSummands);
		virtual double eval(RunningPlan* newRp, RunningPlan* oldRp);
		virtual UtilityInterval* eval(IAssignment* newAss, IAssignment* oldAss);
		void updateAssignment(IAssignment* newAss, IAssignment* oldAss);
		void cacheEvalData();
		void init();
		virtual pair<vector<double>, double>* differentiate(IAssignment* newAss);
		static void initDataStructures();
		virtual string toString();
		Plan* getPlan();
		map<TaskRoleStruct*, double> getPriorityMartix();

		const double DIFFERENCETHRESHOLD = 0.0001;
	protected:
		Plan* plan;
		string name = "DefaultUtilityFunction";
		map<TaskRoleStruct*, double > priorityMartix;
		map<long, double> roleHighestPriorityMap;
		double priorityWeight;
		double similarityWeight;
		AlicaEngine* bpe;
		IRoleAssignment* ra;
		list<USummand*> utilSummands;
		TaskRoleStruct* lookupStruct;
		UtilityInterval* priResult;
		UtilityInterval* getPriorityResult(IAssignment* ass);
		UtilityInterval* simUI;
		UtilityInterval* getSimilarity(IAssignment* newAss, IAssignment* oldAss);

	};

} /* namespace alica */

#endif /* UTILITYFUNCTION_H_ */
