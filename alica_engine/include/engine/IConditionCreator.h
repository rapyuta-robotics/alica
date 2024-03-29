/*
 * IConditionCreator.h
 *
 *  Created on: Oct 8, 2014
 *      Author: Stefan Jakob
 */

#pragma once
#include "engine/BasicCondition.h"

namespace alica
{
class BasicCondition;
struct ConditionContext;

class IConditionCreator
{
public:
    virtual ~IConditionCreator() {}
    virtual std::shared_ptr<BasicCondition> createConditions(int64_t conditionConfId, ConditionContext& conditionContext) = 0;
};

} /* namespace alica */
