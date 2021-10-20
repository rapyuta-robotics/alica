#include "PlanCreator.h"
#include "Master2425328142973735249.h"
#include "Move1889749086610694100.h"
#include "engine/BasicPlan.h"

namespace alica
{

PlanCreator::PlanCreator() {}

PlanCreator::~PlanCreator() {}

std::unique_ptr<BasicPlan> PlanCreator::createPlan(int64_t planId)
{
    switch (planId) {
    case 1889749086610694100:
        return std::make_unique<Move1889749086610694100>();
        break;
    case 2425328142973735249:
        return std::make_unique<Master2425328142973735249>();
        break;
    default:
        std::cerr << "PlanCreator: Unknown plan requested: " << planId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
