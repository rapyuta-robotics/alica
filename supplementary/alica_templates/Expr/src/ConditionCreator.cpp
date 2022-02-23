#include "ConditionCreator.h"
#include "ActionServerExample2379894799421542548.h"
#include "ActionServerExampleMaster2369418759245288160.h"
#include "DummyImplementation4126421719858579722.h"

namespace alica
{

ConditionCreator::ConditionCreator() {}
ConditionCreator::~ConditionCreator() {}

std::shared_ptr<BasicCondition> ConditionCreator::createConditions(int64_t conditionConfId)
{
    switch (conditionConfId) {
    case 587249152722263568:
        return std::make_shared<PreCondition587249152722263568>();
        break;
    case 1886820548377048134:
        return std::make_shared<PreCondition1886820548377048134>();
        break;
    case 2084505765197602547:
        return std::make_shared<PreCondition2084505765197602547>();
        break;
    case 3469760593538210700:
        return std::make_shared<PreCondition3469760593538210700>();
        break;
    default:
        std::cerr << "ConditionCreator: Unknown condition id requested: " << conditionConfId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
