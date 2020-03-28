#include "ConditionCreator.h"
#include "Behaviours/Go2RandomPosition1542881969548.h"
#include "Behaviours/GoTo1544160969061.h"
#include "Master1542881176278.h"
#include "Move1542882005838.h"

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(long conditionConfId)
{
    switch (conditionConfId) {
    case 1542881647180:
        return std::make_shared<PreCondition1542881647180>();
        break;
    case 1542881650423:
        return std::make_shared<PreCondition1542881650423>();
        break;
    case 1543284793605:
        return std::make_shared<RunTimeCondition1543284793605>();
        break;
    default:
        std::cerr << "ConditionCreator: Unknown condition id requested: " << conditionConfId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
