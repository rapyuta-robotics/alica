#include <alica/ConditionCreator.h>
#include <alica/Go2RandomPosition4085572422059465423.h>
#include <alica/GoTo4054297592460872311.h>
#include <alica/Master2425328142973735249.h>
#include <alica/Move1889749086610694100.h>

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(ConditionContext& context)
{
    int64_t conditionConfId = context.conditionConfId;
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
} // namespace alica
