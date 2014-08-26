/*
 * ExpressionHandler.cpp
 *
 *  Created on: Aug 26, 2014
 *      Author: Stefan Jakob
 */

#include "engine/expressionHandler/ExpressionHandler.h"
#include "engine/RunningPlan.h"
#include "engine/model/Plan.h"
#include "engine/model/Transition.h"
#include "engine/model/Condition.h"

namespace alica
{

	ExpressionHandler::ExpressionHandler()
	{
		// TODO Auto-generated constructor stub

	}

	ExpressionHandler::~ExpressionHandler()
	{
		// TODO Auto-generated destructor stub
	}
	//Todo:
	//		void dummyConstraint(ConstraintDescriptor cd, RunningPlan* rp);

	void ExpressionHandler::attachAll()
	{

	}
	bool ExpressionHandler::dummyTrue(RunningPlan* rp)
	{
		return true;
	}
	bool ExpressionHandler::dummyFalse(RunningPlan* rp)
	{
		return false;
	}

//	void ExpressionHandler::attachPlanConditions(Plan* p, T exprType, T consType)
//	{
//
//	}
//	void ExpressionHandler::attachTransConditions(Transition* t, T exprType, T consType)
//	{
//
//	}
//	void ExpressionHandler::attachConstraint(Condition c, T t)
//	{
//
//	}

} /* namespace alica */
