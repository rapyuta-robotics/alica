#pragma once

#include "AlicaElement.h"
#include <memory>

namespace alica
{

class BlackboardBlueprint;
class PlaceholderFactory;

class Placeholder : public AlicaElement
{
public:
    const BlackboardBlueprint* getBlackboardBlueprint() const { return _blackboardBlueprint.get(); }

private:
    friend PlaceholderFactory;
    /**
     * The blackboard mapping. This cannot be null, since placeholders cannot inherit the parent's blackboard
     */
    std::unique_ptr<BlackboardBlueprint> _blackboardBlueprint;
};

} // namespace alica
