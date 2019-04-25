#include "alica_viewer/alica_plan_parser.h"
#include <SystemConfig.h>
#include <assert.h>
#include <engine/AlicaClock.h>

namespace alica
{
namespace
{
void sortPlanTrees(std::vector<std::unique_ptr<PlanTree>>& planTrees)
{
    std::sort(planTrees.begin(), planTrees.end(), [](const std::unique_ptr<PlanTree>& a, const std::unique_ptr<PlanTree>& b) {
        if (a->getEntryPoint() == b->getEntryPoint()) {
            return a->getState()->getId() < b->getState()->getId();
        } else {
            return a->getEntryPoint()->getId() < b->getEntryPoint()->getId();
        }
    });
}
} // namespace

PlanTree::PlanTree(AlicaTime creationTime)
    : _parent(nullptr)
    , _state(nullptr)
    , _entryPoint(nullptr)
    , _numOfChildren(0)
    , _time(creationTime)
    , _x(0)
    , _y(0)
{
}

PlanTree::PlanTree(const PlanTree& other, const PlanTree& parent)
    : _state(other._state)
    , _entryPoint(other._entryPoint)
    , _robotIds(other._robotIds)
    , _time(other._time)
    , _numOfChildren(0)
    , _x(other._x)
    , _y(other._y)
{
    setParent(parent);
    for (const auto& ptvMapPair : other._children) {
        for (const auto& child : ptvMapPair.second) {
            addChildren(std::unique_ptr<PlanTree>(new PlanTree(*child, *this)));
        }
    }
}

bool PlanTree::operator==(const PlanTree& other) const
{
    if (_state == nullptr || other._state == nullptr) {
        return false;
    }
    return _state == other._state;
}

void PlanTree::setParent(const PlanTree& parent)
{
    _parent = &parent;
    _x = parent._x + parent._numOfChildren;
    _y = parent._y + 1;
}

bool PlanTree::setState(const State* state)
{
    if (state == nullptr) {
        std::cout << "Unknown state." << std::endl;
        return false;
    }
    const EntryPoint* entryPoint = nullptr;
    for (const EntryPoint* ep : state->getInPlan()->getEntryPoints()) {
        if (std::find(ep->getReachableStates().begin(), ep->getReachableStates().end(), state) != ep->getReachableStates().end()) {
            entryPoint = ep;
        }
    }
    if (entryPoint == nullptr) {
        std::cout << "Entrypoint unknown for state (" << state << ")." << std::endl;
        return false;
    }
    _state = state;
    _entryPoint = entryPoint;
    return true;
}

void PlanTree::addChildren(std::unique_ptr<PlanTree> child)
{
    if (!child || !child->isValid()) {
        std::cerr << "Child not added. \n";
        return;
    }

    int64_t planId = child->getEntryPoint()->getPlan()->getId();
    PlanTreeVectorMap::iterator ptvEntry = _children.find(planId);
    if (ptvEntry != _children.end()) {
        ptvEntry->second.push_back(std::move(child));
        sortPlanTrees(ptvEntry->second);
    } else {
        std::vector<std::unique_ptr<PlanTree>> childVector;
        childVector.push_back(std::move(child));
        _children.emplace(planId, std::move(childVector));
    }
    ++_numOfChildren;
}

void PlanTree::addRobot(essentials::AgentIDConstPtr robotId)
{
    if (robotId) {
        _robotIds.push_back(robotId);
    }
}

void PlanTree::mergeRobots(const AgentGrp& robotIds)
{
    // Here we assume that no duplicate robotID are present
    for (essentials::AgentIDConstPtr robotId : robotIds) {
        addRobot(robotId);
    }
}

void PlanTree::mergePlanTree(const PlanTree& src)
{
    if (!src.isValid()) {
        return;
    }
    if (!_children.empty()) {
        int64_t planId = src.getEntryPoint()->getPlan()->getId();
        PlanTreeVectorMap::iterator ptvEntry = _children.find(planId);
        if (ptvEntry != _children.end()) {
            std::vector<std::unique_ptr<PlanTree>>::iterator iter =
                std::find_if(ptvEntry->second.begin(), ptvEntry->second.end(), [&](const std::unique_ptr<PlanTree>& pt) { return *pt == src; });
            if (iter != ptvEntry->second.end()) {
                (*iter)->mergeRobots(src.getRobots());
                for (const auto& ptvMapPair : src._children) {
                    for (const auto& child : ptvMapPair.second) {
                        (*iter)->mergePlanTree(*child);
                    }
                }
                sortPlanTrees(ptvEntry->second);
                return;
            }
        }
    }
    addChildren(std::unique_ptr<PlanTree>(new PlanTree(src, *this)));
}

void PlanTree::getRobotsSorted(AgentGrp& robotIds) const
{
    robotIds = _robotIds;
    std::sort(robotIds.begin(), robotIds.end());
}

AlicaPlan::AlicaPlan(int argc, char* argv[])
    : _modelManager(&_planRepository)
    , _combinedPlanTree(nullptr)
    , _agentIDManager(new essentials::AgentIDFactory())
{
    if (argc < 2) {
        std::cout << "Usage: Base -m [Masterplan] -r [roleset]" << std::endl;
        exit(1);
    }

    std::string masterPlanName = "";
    std::string roleSetName = "";

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-m" || std::string(argv[i]) == "-masterplan") {
            masterPlanName = argv[++i];
        }
        if (std::string(argv[i]) == "-r" || std::string(argv[i]) == "-roleset") {
            roleSetName = argv[++i];
        }
    }

    if (masterPlanName.size() == 0) {
        std::cout << "Usage: Base -m [Masterplan] -r [roleset]" << std::endl;
        exit(1);
    }
    std::cout << "Masterplan is: " << masterPlanName << std::endl;
    std::cout << "Rolset is: " << roleSetName << std::endl;

    _modelManager.loadPlanTree(masterPlanName);

    _teamTimeOut = AlicaTime::milliseconds((*essentials::SystemConfig::getInstance())["Alica"]->get<uint64_t>("Alica.TeamTimeOut", NULL));
}

void AlicaPlan::combinePlanTree(PlanTree& planTree) const
{
    AlicaTime oldestTime = _clock.now() - _teamTimeOut;
    for (const auto& ptMapPair : _planTrees) {
        if (ptMapPair.second->getCreationTime() > oldestTime) {
            planTree.mergePlanTree(*ptMapPair.second);
        }
    }
}

void AlicaPlan::handlePlanTreeInfo(const PlanTreeInfo& incoming)
{
    std::unique_ptr<PlanTree> pt = planTreeFromMessage(incoming.senderID, incoming.stateIDs);
    if (pt) {
        PlanTreeMap::iterator ptEntry = _planTrees.find(incoming.senderID);
        if (ptEntry != _planTrees.end()) {
            ptEntry->second = std::move(pt);
        } else {
            _planTrees.emplace(incoming.senderID, std::move(pt));
            if (_agentInfos.find(incoming.senderID) == _agentInfos.end()) {
                std::stringstream s;
                s << incoming.senderID;
                _agentInfos.emplace(incoming.senderID, AgentInfo(incoming.senderID, s.str()));
            }
        }
    }
}

std::unique_ptr<PlanTree> AlicaPlan::planTreeFromMessage(essentials::AgentIDConstPtr robotId, const IdGrp& ids)
{
    if (ids.empty()) {
        std::cerr << "Empty state list for robot " << robotId << std::endl;
        return std::unique_ptr<PlanTree>{};
    }
    AlicaTime now = _clock.now();
    std::unique_ptr<PlanTree> root(new PlanTree(now));
    root->addRobot(robotId);

    IdGrp::const_iterator iter = ids.begin();
    const PlanRepository::Accessor<State>& validStates = _planRepository.getStates();

    if (!root->setState(validStates.find(*iter))) {
        std::cout << "Unable to add State (" << *iter << ") received from " << *robotId << std::endl;
        return std::unique_ptr<PlanTree>{};
    }

    if (ids.size() <= 1) {
        return root;
    }

    ++iter;
    PlanTree* curParent = nullptr;
    PlanTree* cur = root.get();
    for (; iter != ids.end(); ++iter) {
        if (*iter == -1) {
            curParent = cur;
            cur = nullptr;
        } else if (*iter == -2) {
            cur = curParent;
            if (cur == nullptr) {
                std::cout << "Malformed SptMessage from " << *robotId << std::endl;
                return std::unique_ptr<PlanTree>{};
            }
        } else {
            std::unique_ptr<PlanTree> node(new PlanTree(now));
            node->addRobot(robotId);
            if (!node->setState(validStates.find(*iter))) {
                std::cout << "Unable to add State (" << *iter << ") received from " << *robotId << std::endl;
                return std::unique_ptr<PlanTree>{};
            }
            cur = node.get();
            assert(curParent != nullptr);
            node->setParent(*curParent);
            curParent->addChildren(std::move(node));
        }
    }
    return root;
}

const AgentInfo* AlicaPlan::getAgentInfo(essentials::AgentIDConstPtr agentID) const
{
    AgentInfoMap::const_iterator agentInfoEntry = _agentInfos.find(agentID);
    if (agentInfoEntry != _agentInfos.end()) {
        return &agentInfoEntry->second;
    }
    return nullptr;
}

} // namespace alica
