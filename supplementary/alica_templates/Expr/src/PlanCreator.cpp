#include "PlanCreator.h"
#include "ActionServerExample2379894799421542548.h"
#include "ActionServerExampleMaster2369418759245288160.h"
#include "DummyImplementation4126421719858579722.h"
#include "engine/BasicPlan.h"

namespace alica
{

PlanCreator::PlanCreator() {}

PlanCreator::~PlanCreator() {}

std::unique_ptr<BasicPlan> PlanCreator::createPlan(int64_t planId, PlanContext& context)
{
    switch (planId) {
    case 2369418759245288160:
        return std::make_unique<ActionServerExampleMaster2369418759245288160>(context);
        break;
    case 2379894799421542548:
        return std::make_unique<ActionServerExample2379894799421542548>(context);
        break;
    case 4126421719858579722:
        return std::make_unique<DummyImplementation4126421719858579722>(context);
        break;
    default:
        std::cerr << "PlanCreator: Unknown plan requested: " << planId << std::endl;
        throw new std::exception();
        break;
    }
}
} // namespace alica
