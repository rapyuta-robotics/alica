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
     * If nullptr, it will simply receive a reference to its parents Blackboard
     * Otherwise, the mapped keys will be copied in and out of the plans Blackboard
     */
    std::unique_ptr<BlackboardBlueprint> _blackboardBlueprint;
};

} // namespace alica
