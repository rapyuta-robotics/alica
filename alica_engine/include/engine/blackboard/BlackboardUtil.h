#include "engine/blackboard/KeyMapping.h"
#include "engine/blackboard/Blackboard.h"

namespace alica
{
class BlackboardUtil
{
public:
    static void setInput(const Blackboard* parent_bb, Blackboard* child_bb, const KeyMapping* keyMapping);
    static void setOutput(Blackboard* parent_bb, const Blackboard* child_bb, const KeyMapping* keyMapping);
};
}