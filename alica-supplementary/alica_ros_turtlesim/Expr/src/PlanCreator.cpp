#include "PlanCreator.h"
#include "Master1542881176278.h"
#include "Move1542882005838.h"
#include "engine/BasicPlan.h"

namespace alica
{

PlanCreator::PlanCreator() {}

PlanCreator::~PlanCreator() {}

std::unique_ptr<BasicPlan> PlanCreator::createPlan(int64_t planId)
{
    switch (planId) {
    case 1542881176278:
        return std::make_unique<Master1542881176278>();
        break;
    case 1542882005838:
        return std::make_unique<Move1542882005838>();
        break;
    default:
        std::cerr << "PlanCreator: Unknown plan requested: " << planId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
