#include "Master.h"

namespace alica
{

Master::Master(PlanContext& context)
        : BasicPlan(context)
{
    std::cerr << "Masterr created" << std::endl;
}
Master::~Master() {}

} // namespace alica
