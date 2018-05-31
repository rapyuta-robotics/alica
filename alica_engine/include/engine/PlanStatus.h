#pragma once

namespace alica
{
/**
 * Reflects the status of a RunningPlan
 */
enum class PlanStatus
{
    Running,
    Success,
    Failed,
    Retired
};
const char* getPlanStatusName(PlanStatus ps)
{
    switch (ps) {
    case PlanStatus::Running:
        return "Running";
    case PlanStatus::Success:
        return "Success";
    case PlanStatus::Failed:
        return "Failed";
    case PlanStatus::Retired:
        return "Retired";
    }
    return "Undefined";
}

} /* namespace alica */
