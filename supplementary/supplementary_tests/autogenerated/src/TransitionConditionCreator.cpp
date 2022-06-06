#include "supplementary_tests/TransitionConditionCreator.h"

#include "supplementary_tests/conditions/conditions.h"
#include <iostream>
#include <engine/blackboard/Blackboard.h>
#include <engine/RunningPlan.h>
#include <engine/IAlicaWorldModel.h>

namespace alica
{

TransitionConditionCreator::TransitionConditionCreator() {}

TransitionConditionCreator::~TransitionConditionCreator() {}

std::function<bool(const Blackboard*, const RunningPlan*, const IAlicaWorldModel*)> TransitionConditionCreator::createConditions(int64_t conditionId)
{
    switch (conditionId) {
    case 295816226925111421:
        return std::bind(conditionMISSING_NAME1479557592662, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    default:
        std::cerr << "TransitionConditionCreator: Unknown condition id requested: " << conditionId << std::endl;
        throw new std::exception();
        break;
    }
}
} /* namespace alica */
