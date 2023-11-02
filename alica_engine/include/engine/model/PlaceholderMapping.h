#pragma once

#include "AlicaElement.h"
#include <unordered_map>

namespace alica
{

class PlaceholderMappingFactory;
class AbstractPlan;

class PlaceholderMapping : public AlicaElement
{
public:
    PlaceholderMapping() = default;
    ~PlaceholderMapping() = default;

    const AbstractPlan* find(int64_t placeholderId) const;

private:
    friend PlaceholderMappingFactory;

    std::unordered_map<int64_t, const AbstractPlan*> _mapping;
};

} // namespace alica
