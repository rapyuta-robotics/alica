#pragma once
#include "engine/Types.h"

#include <vector>
#include <utility>

namespace alica
{

class AlicaEngine;

/**
 * Globally holds information about succeeded entry points for a specific robot
 */
class SuccessMarks
{
public:
    SuccessMarks();
    ~SuccessMarks();

    void limitToContexts(const std::vector<std::pair<std::size_t, const AbstractPlan*>>& activeContexts);
    void fromMsg(const AlicaEngine* ae, const std::vector<std::size_t>& msg);

    void clear();
    EntryPointGrp succeededEntryPoints(std::size_t parentContextHash, const AbstractPlan* p) const;
    void removePlan(std::size_t parentContextHash, const AbstractPlan* plan);
    void markSuccessful(std::size_t parentContextHash, const EntryPoint* e);

    bool succeeded(std::size_t parentContextHash, const EntryPoint* e) const;
    bool anyTaskSucceeded(std::size_t parentContextHash, const AbstractPlan* p) const;
    std::vector<std::size_t> toMsg() const;

private:
};

} /* namespace alica */
