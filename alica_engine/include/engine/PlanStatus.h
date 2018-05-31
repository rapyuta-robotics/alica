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
    case Running:
        return "Running";
    case Success:
        return "Success";
    case Failed:
        return "Failed";
    case Retired:
        return "Retired";
    }
}

} /* namespace alica */
