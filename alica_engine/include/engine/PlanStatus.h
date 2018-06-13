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
    Failed
};
enum class PlanActivity
{
    InActive,
    Active,
    Retired
};

inline const char* getPlanStatusName(PlanStatus ps)
{
    switch (ps) {
    case PlanStatus::Running:
        return "Running";
    case PlanStatus::Success:
        return "Success";
    case PlanStatus::Failed:
        return "Failed";
    }
    return "Undefined";
}

inline const char* getPlanActivityName(PlanActivity pa)
{
    switch (pa) {
    case PlanActivity::InActive:
        return "InActive";
    case PlanActivity::Active:
        return "Active";
    case PlanActivity::Retired:
        return "Retired";
    }
    return "Undefined";
}

} /* namespace alica */
