#include "ConditionCreator.h"
#include "Go2RandomPosition4085572422059465423.h"
#include "GoTo4054297592460872311.h"
#include "Master2425328142973735249.h"
#include "Move1889749086610694100.h"

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(int64_t conditionConfId)
{
    switch (conditionConfId) {
    case 1136497454350831106:
        return std::make_shared<PreCondition1136497454350831106>();
        break;
    case 1288817888979746811:
        return std::make_shared<RunTimeCondition1288817888979746811>();
        break;
    case 1597434482701133956:
        return std::make_shared<PreCondition1597434482701133956>();
        break;
    default:
        std::cerr << "ConditionCreator: Unknown condition id requested: " << conditionConfId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
