#include <alica/ConditionCreator.h>
#include <alica/Master.h>
#include <alica/Move.h>

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(int64_t conditionConfId, ConditionContext& context)
{
    switch (conditionConfId) {
    case 1288817888979746811:
        return std::make_shared<MoveRunTimeCondition>();
        break;
    default:
        std::cerr << "ConditionCreator: Unknown condition id requested: " << conditionConfId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
