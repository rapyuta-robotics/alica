#include <engine/model/PlaceholderMapping.h>

namespace alica
{

const AbstractPlan* PlaceholderMapping::find(int64_t placeholderId) const
{
    if (auto abstractPlan = _mapping.find(placeholderId); abstractPlan != _mapping.end()) {
        return abstractPlan->second;
    }
    return nullptr;
}

} // namespace alica
