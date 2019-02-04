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
class Condition;
class PlanRepository;
class AlicaCreators;
class IConstraintCreator;

/**
 * The ExpressionHandler attaches expressions and constraints to plans during start-up of the engine.
 */
class ExpressionHandler
{
public:
    ExpressionHandler();
    virtual ~ExpressionHandler();
    void attachAll(PlanRepository& pr, AlicaCreators& creatorCtx);

private:
    void attachConstraint(Condition* c, IConstraintCreator& crc);
};

} /* namespace alica */

#endif /* EXPRESSIONHANDLER_H_ */
