#include "engine/BasicPlan.h"
#include <alica/Master.h>
#include <alica/Move.h>
#include <alica/PlanCreator.h>

namespace alica
{

PlanCreator::PlanCreator() {}

PlanCreator::~PlanCreator() {}

std::unique_ptr<BasicPlan> PlanCreator::createPlan(int64_t planId, PlanContext& context)
{
    switch (planId) {
    case 1889749086610694100:
        return std::make_unique<Move>(context);
        break;
    case 2425328142973735249:
        return std::make_unique<Master>(context);
        break;
    default:
        std::cerr << "PlanCreator: Unknown plan requested: " << planId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
