/*
 * ExpressionHandler.h
 *
 *  Created on: Aug 26, 2014
 *      Author: Stefan Jakob
 */

#ifndef EXPRESSIONHANDLER_H_
#define EXPRESSIONHANDLER_H_

namespace alica
{
class RunningPlan;
class Plan;
class Condition;
class Transition;
class IConditionCreator;
class IConstraintCreator;
class IUtilityCreator;
class AlicaEngine;

/**
 * The ExpressionHandler attaches expressions and constraints to plans during start-up of the engine.
 */
class ExpressionHandler
{
public:
    ExpressionHandler(AlicaEngine* ae);
    virtual ~ExpressionHandler();
    void attachAll(IConditionCreator& cc, IUtilityCreator& uc, IConstraintCreator& crc);
    //		void dummyConstraint(ConstraintDescriptor cd, RunningPlan* rp);
    bool dummyTrue(RunningPlan* rp);
    bool dummyFalse(RunningPlan* rp);

private:
    AlicaEngine* _ae;
    void attachConstraint(Condition* c, IConstraintCreator& crc);
};

} /* namespace alica */

#endif /* EXPRESSIONHANDLER_H_ */
