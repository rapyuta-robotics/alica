
namespace alica
{
class Blackboard;
class KeyMapping;

class BlackboardUtil
{
public:
    static void setInput(const Blackboard* parent_bb, Blackboard* child_bb, const KeyMapping* keyMapping);
    static void setOutput(Blackboard* parent_bb, const Blackboard* child_bb, const KeyMapping* keyMapping);
};
} // namespace alica
