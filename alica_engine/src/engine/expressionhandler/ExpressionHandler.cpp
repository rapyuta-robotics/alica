/*
 * ExpressionHandler.cpp
 *
 *  Created on: Aug 26, 2014
 *      Author: Stefan Jakob
 */

#include "engine/expressionhandler/ExpressionHandler.h"
#include "engine/RunningPlan.h"
#include "engine/model/Plan.h"
#include "engine/model/Transition.h"
#include "engine/model/Condition.h"
#include <SystemConfig.h>
#include "engine/IConditionCreator.h"
#include "engine/IUtilityCreator.h"
#include "engine/IConstraintCreator.h"
#include "engine/BasicUtilityFunction.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/PreCondition.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/expressionhandler/BasicFalseCondition.h"
#include "engine/expressionhandler/BasicTrueCondition.h"
#include "engine/expressionhandler/DummyConstraint.h"

namespace alica
{

	/**
	 * Constructor, loads the assembly containing expressions and constraints.
	 */
	ExpressionHandler::ExpressionHandler(AlicaEngine* ae, IConditionCreator* cc, IUtilityCreator* uc,
											IConstraintCreator* crc)
	{
		this->ae = ae;
		this->conditionCreator = cc;
		this->utilityCreator = uc;
		this->constraintCreator = crc;
	}

	ExpressionHandler::~ExpressionHandler()
	{
		// TODO Auto-generated destructor stub
	}

	/**
	 * Attaches expressions and constraints to the plans. Called by the AlicaEngine during start up.
	 */
	void ExpressionHandler::attachAll()
	{
		PlanRepository* pr = ae->getPlanRepository();
		for (auto it : pr->getPlans())
		{
			auto p = it.second;

			auto ufGen = utilityCreator->createUtility(p->getId());
			//TODO hole utility function (siehe c#)
			p->setUtilityFunction(ufGen->getUtilityFunction(p));

			if (p->getPreCondition() != nullptr)
			{
				if (p->getPreCondition()->isEnabled())
				{
					p->getPreCondition()->setBasicCondition(
							this->conditionCreator->createConditions(p->getPreCondition()->getId()));
					attachConstraint(p->getPreCondition());
				}
				else
				{
					p->getPreCondition()->setBasicCondition(make_shared<BasicFalseCondition>());
				}
			}

			if (p->getRuntimeCondition() != nullptr)
			{
				p->getRuntimeCondition()->setBasicCondition(
						this->conditionCreator->createConditions(p->getRuntimeCondition()->getId()));
				attachConstraint(p->getRuntimeCondition());
			}

			for (auto t : p->getTransitions())
			{
				if (t->getPreCondition() != nullptr)
				{
					if (t->getPreCondition()->isEnabled())
					{
						t->getPreCondition()->setBasicCondition(
								this->conditionCreator->createConditions(t->getPreCondition()->getId()));
						attachConstraint(t->getPreCondition());
					}
					else
					{
						t->getPreCondition()->setBasicCondition(make_shared<BasicFalseCondition>());
					}
				}
			}
		}
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

	void ExpressionHandler::attachConstraint(Condition* c)
	{
		if (c->getVariables().size() == 0 && c->getQuantifiers().size() == 0)
		{
			c->setBasicConstraint(make_shared<DummyConstraint>());
		}
		else
		{
			c->setBasicConstraint(this->constraintCreator->createConstraint(c->getId()));
		}
	}

} /* namespace alica */
