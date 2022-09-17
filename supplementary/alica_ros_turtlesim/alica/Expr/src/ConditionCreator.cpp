#include <alica/ConditionCreator.h>
#include <alica/Other.h>

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(int64_t conditionConfId)
{
    switch (conditionConfId) {
    case 1288817888979746811:
        return std::make_shared<RunTimeCondition1288817888979746811>();
        break;
    default:
        std::cerr << "ConditionCreator: Unknown condition id requested: " << conditionConfId << std::endl;
        throw new std::exception();
        break;
    }
}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(ConditionContext& conditionContext)
{
    return nullptr;
}

} // namespace alica
