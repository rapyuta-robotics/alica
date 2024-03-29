#include "engine/model/PlanType.h"
#include "engine/model/Plan.h"
#include <sstream>

namespace alica
{
PlanType::PlanType()
        : AbstractPlan()
{
}

PlanType::~PlanType() {}

const Plan* PlanType::getPlanById(int64_t id) const
{
    for (const Plan* p : _plans) {
        if (p->getId() == id) {
            return p;
        }
    }
    return nullptr;
}

std::string PlanType::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#PlanType: " << getName() << " " << getId() << std::endl;
    ss << indent << "\t Plans: " << _plans.size() << std::endl;
    if (_plans.size() != 0) {
        for (const Plan* p : _plans) {
            ss << indent << "\t" << p->getId() << " " << p->getName() << std::endl;
        }
    }
    ss << indent << "#EndPlanType" << std::endl;
    return ss.str();
}

} // namespace alica
