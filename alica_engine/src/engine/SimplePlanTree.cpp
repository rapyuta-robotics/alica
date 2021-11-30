
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/model/Task.h"
#include "engine/Types.h"
#include <engine/SimplePlanTree.h>
#include <engine/util/HashFunctions.h>

namespace alica
{

SimplePlanTree::SimplePlanTree()
        : _agentId(InvalidAgentID)
        , _parent(nullptr)
        , _state(nullptr)
        , _entryPoint(nullptr)
        , _isNew(true)
{
}

SimplePlanTree::~SimplePlanTree() {}

void SimplePlanTree::computeContextHash()
{
    _contextHash = contextHash(_parent ? _parent->_contextHash : contextHash(0), _entryPoint->getDynamicId(), _state->getId());
}

bool SimplePlanTree::containsContexts(const std::unordered_set<std::size_t>& contextHashes) const
{
    if (contextHashes.find(contextHash(_contextHash, _entryPoint->getId(), _entryPoint->getDynamicId())) != contextHashes.end()) {
        return true;
    }
    auto found = false;
    for (auto& child : _children) {
        found = child->containsContexts(contextHashes);
    }
    return found;
}

bool SimplePlanTree::containsContext(std::size_t parentContextHash, const AbstractPlan* plan) const
{
    auto p = dynamic_cast<const Plan*>(plan);
    if (!p) {
        return false;
    }

    std::unordered_set<std::size_t> contextHashes;
    for (const auto ep : p->getEntryPoints()) {
        contextHashes.insert(contextHash(parentContextHash, ep->getId(), ep->getDynamicId()));
    }

    return containsContexts(contextHashes);
}

void SimplePlanTree::setEntryPoint(const EntryPoint* entryPoint)
{
    _entryPoint = entryPoint;
}

void SimplePlanTree::setState(const State* state)
{
    _state = state;
}

void SimplePlanTree::setReceiveTime(AlicaTime receiveTime)
{
    _receiveTime = receiveTime;
}

void SimplePlanTree::setDynamicStateIDPairs(const IdGrp& dynamicStateIdPairs)
{
    _dynamicStateIdPairs = dynamicStateIdPairs;
}

std::ostream& operator<<(std::ostream& out, const SimplePlanTree& spt)
{
    out << "RobotID: " << spt._agentId << "\n";
    out << "Parent: ";

    if (spt._parent != nullptr) {
        out << spt._parent->getState()->getId();
    }
    out << std::endl;

    out << "State: ";
    if (spt._state != nullptr) {
        out << spt._state->getId();
        out << " " + spt._state->getName();
    } else {
        out << "ERROR !!!NO STATE !!!";
    }

    out << std::endl;

    out << "EntryPoint: ";

    if (spt._entryPoint != nullptr) {
        out << spt._entryPoint->getId() << " " << spt._entryPoint->getTask()->getName();
    } else {
        out << "NoEntryPoint";
    }
    out << std::endl;

    out << "Children: " << spt._children.size() << "\n";
    for (const std::unique_ptr<SimplePlanTree>& c : spt._children) {
        out << *c;
    }

    out << std::endl << std::endl;

    return out;
}

} /* namespace alica */
