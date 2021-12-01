#include "engine/collections/SuccessMarks.h"

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"

namespace alica
{

/**
 * Default Constructor
 */
SuccessMarks::SuccessMarks() {}

SuccessMarks::~SuccessMarks() {}

/**
 * Update from the vector which is a flattened list of tuples: <parentContextHash, entrypoint id, dynamic entry point id>
 */
void SuccessMarks::fromMsg(const AlicaEngine* ae, const std::vector<std::size_t>& msg)
{
    clear();
    if (msg.size() % 3) {
        ALICA_WARNING_MSG("Invalid success marks: msg size is not a multiple of 3");
        return;
    }
    for (int i = 0; i < static_cast<int>(msg.size()); i += 3) {
        auto ep = ae->getEntryPointStore()->get(msg[i + 1], msg[i + 2]);
        _successMarks[{msg[i], ep->getPlan()->getId()}].emplace_back(ep);
    }
}

/**
 * Drop every success mark not occurring in the active contexts
 */
void SuccessMarks::limitToContexts(const std::vector<std::pair<std::size_t, int64_t>>& activeContexts)
{
    std::vector<std::pair<std::size_t, int64_t>> toRemove;
    for (const auto& successMarkEntry : _successMarks) {
        auto it = std::find(activeContexts.begin(), activeContexts.end(), successMarkEntry.first);
        if (it == activeContexts.end()) {
            toRemove.push_back(successMarkEntry.first);
        }
    }
    for (auto p : toRemove) {
        _successMarks.erase(p);
    }
}

/**
 * Clear all marks
 */
void SuccessMarks::clear()
{
    _successMarks.clear();
}

/**
 * Get all EntryPoints succeeded in the given parent context corresponding to the planId
 */
EntryPointGrp SuccessMarks::succeededEntryPoints(std::size_t parentContextHash, int64_t planId) const
{
    auto it = _successMarks.find({parentContextHash, planId});
    if (it == _successMarks.end()) {
        return EntryPointGrp{};
    }
    return it->second;
}

/**
 * Remove all marks referring to the specified plan in the given parent context
 */
void SuccessMarks::removePlan(std::size_t parentContextHash, int64_t planId)
{
    _successMarks.erase({parentContextHash, planId});
}

/**
 * Mark an EntryPoint within a plan as successfully completed
 */
void SuccessMarks::markSuccessful(std::size_t parentContextHash, const EntryPoint* ep)
{
    _successMarks[{parentContextHash, ep->getPlan()->getId()}].emplace_back(ep);
}

/**
 * Serialize to a vector of EntryPoint ids.
 */
std::vector<std::size_t> SuccessMarks::toContextGrp() const
{
    return std::vector<std::size_t>(_successMarks.begin(), _successMarks.end());
}

} /* namespace alica */
