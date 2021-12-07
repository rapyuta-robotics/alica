#pragma once
#include "engine/Types.h"
#include "engine/util/HashFunctions.h"

#include <vector>
#include <utility>
#include <unordered_map>

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

    void limitToContexts(const std::vector<std::pair<std::size_t, int64_t>>& activeContexts);
    void fromMsg(const AlicaEngine* ae, const std::vector<std::size_t>& msg);

    void clear();
    EntryPointGrp succeededEntryPoints(std::size_t parentContextHash, int64_t planId) const;
    void removePlan(std::size_t parentContextHash, int64_t planId);
    void markSuccessful(std::size_t parentContextHash, const EntryPoint* ep);

    std::vector<std::size_t> toMsg() const;

private:
    struct IDHash
    {
        std::size_t operator()(std::pair<std::size_t, int64_t> context) const
        {
            return hashCombine(context.first, contextHash(context.second));
        }
    };

    std::unordered_map<std::pair<std::size_t, int64_t>, EntryPointGrp, IDHash> _successMarks;
};

} /* namespace alica */
