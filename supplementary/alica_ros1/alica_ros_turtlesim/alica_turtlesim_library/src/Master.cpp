#include "Master.h"

namespace alica
{

Master::Master(PlanContext& context)
        : BasicPlan(context)
{
    std::cerr << "Master created" << std::endl;
}
Master::~Master() {}

void Master::onInit() {}

} // namespace alica
