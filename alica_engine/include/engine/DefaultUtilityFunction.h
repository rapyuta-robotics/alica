/*
 * DefaultUtilityFunction.h
 *
 *  Created on: Jul 30, 2014
 *      Author: Stefan Jakob
 */

#ifndef DEFAULTUTILITYFUNCTION_H_
#define DEFAULTUTILITYFUNCTION_H_

using namespace std;

#include <string>
#include <sstream>

#include "engine/UtilityFunction.h"

namespace alica
{

	class Plan;
	class RunningPlan;
	class UtilityInterval;

	/**
	 * A default implementation for a plan's utility function. The only occuring summand referrs to the task-role preferences.
	 */
	class DefaultUtilityFunction : virtual public UtilityFunction
	{
	public:
		DefaultUtilityFunction(Plan* plan);
		virtual ~DefaultUtilityFunction();
		double eval(RunningPlan* newRp, RunningPlan* oldRp);
		UtilityInterval* eval (IAssignment* newAss, IAssignment* oldAss);
		string toString();

	};

} /* namespace alica */

#endif /* DEFAULTUTILITYFUNCTION_H_ */
