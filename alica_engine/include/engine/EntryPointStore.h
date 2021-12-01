#pragma once

#include <engine/util/LRUCache.h>
#include <engine/util/HashFunctions.h>
#include <unordered_map>
#include <cassert>

namespace alica
{

class EntryPoint;

class EntryPointStore
{
public:
    template <class ForwardIt>
    EntryPointStore(ForwardIt staticEntryPointsFirst, ForwardIt staticEntryPointsLast)
            : _staticEntryPoints(staticEntryPointsFirst, staticEntryPointsLast)
            , _dynamicEntryPoints(1000)
    {
        // TODO: get the LRU cache size from the alica config
    }

    // Get the entry point with the given staticEntryPointId & dynamicEntryPointId
    // If dynamicEntryPointId > 0 the entry point should be dynamic
    // If the dynamic entry point does not exist, it is dynamically created
    const EntryPoint* get(int64_t staticEntryPointId, int64_t dynamicEntryPointId = 0) const
    {
        auto staticEp = getStaticEp(staticEntryPointId);
        assert(staticEp);
        if (!dynamicEntryPointId) {
            return staticEp;
        }
        assert(staticEp->isDynamic());
        return std::addressof(_dynamicEntryPoints.lookup({staticEntryPointId, dynamicEntryPointId}, *staticEp, dynamicEntryPointId));
    }

private:
    const EntryPoint* getStaticEp(int64_t staticEntryPointId) const
    {
        auto it = _staticEntryPoints.find(staticEntryPointId);
        assert(it != _staticEntryPoints.end());
        return it->second;
    }

    struct IDHash
    {
        std::size_t operator()(std::pair<int64_t, int64_t> p) const
        {
            return hashCombine(contextHash(p.first), contextHash(p.second));
        }
    };

    const std::unordered_map<int64_t, const EntryPoint*> _staticEntryPoints;
    mutable LRUCache<std::pair<int64_t, int64_t>, EntryPoint, IDHash> _dynamicEntryPoints;
};

} // namespace alica