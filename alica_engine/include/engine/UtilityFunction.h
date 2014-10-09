/*
 * UtilityFunction.h
 *
 *  Created on: Jun 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef UTILITYFUNCTION_H_
#define UTILITYFUNCTION_H_
//#define UFDEBUG

using namespace std;

#include <string>
#include <map>
#include <list>
#include <algorithm>
#include <vector>
#include <sstream>
#include <memory>

#include "engine/RunningPlan.h"

namespace alica
{
	class Plan;
	class AlicaEngine;
	class IRoleAssignment;
	class USummand;
	class IAssignment;
	struct UtilityInterval;
	struct TaskRoleStruct;

	class UtilityFunction
	{
	public:
		UtilityFunction(string name, list<USummand*> utilSummands, double priorityWeight, double similarityWeight, Plan* plan);
		virtual ~UtilityFunction();
		list<USummand*>& getUtilSummands();
		void setUtilSummands(list<USummand*> utilSummands);
		virtual double eval(shared_ptr<RunningPlan> newRp, shared_ptr<RunningPlan> oldRp);
		virtual UtilityInterval* eval(IAssignment* newAss, IAssignment* oldAss);
		void updateAssignment(IAssignment* newAss, IAssignment* oldAss);
		void cacheEvalData();
		void init();
		virtual pair<vector<double>, double>* differentiate(IAssignment* newAss);
		static void initDataStructures();
		virtual string toString();
		Plan* getPlan();
		map<TaskRoleStruct*, double>& getPriorityMartix();

		const double DIFFERENCETHRESHOLD = 0.0001; // Max difference for the same result
	protected:
		Plan* plan;
		string name = "DefaultUtilityFunction";
		// For default priority based utility summand (which is integrated in every UF)
		map<TaskRoleStruct*, double > priorityMartix;
		map<long, double> roleHighestPriorityMap;
		// For default similarity based utility summand (which is integrated in every UF)
		double priorityWeight;
		double similarityWeight;
		AlicaEngine* ae;
		IRoleAssignment* ra;
		// List of normal utility summands
		list<USummand*> utilSummands;
		TaskRoleStruct* lookupStruct;
		UtilityInterval* priResult;
		UtilityInterval* getPriorityResult(IAssignment* ass);
		UtilityInterval* simUI;
		UtilityInterval* getSimilarity(IAssignment* newAss, IAssignment* oldAss);

	};

} /* namespace alica */

#endif /* UTILITYFUNCTION_H_ */
