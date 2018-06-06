#include "engine/planselector/IAssignment.h"

#include <engine/model/Plan.h>

namespace alica
{

AssignmentSuccessIterator AssignmentSuccessView::end() const
{
    const AgentGrp* agents = _pas->getProblem()->getSuccess(pas->getPlan())->getAgentsByIndex(_epidx);
    return AssignmentSuccessIterator(agents ? agents->size() : 0, true, _epidx, _pas);
}

UniqueAssignmentSuccessIterator UniqueAssignmentSuccessView::end() const
{
    const AgentGrp* agents = _pas->getProblem()->getSuccess(pas->getPlan())->getAgentsByIndex(_epidx);
    return UniqueAssignmentSuccessIteraotr(agents ? agents->size() : 0, true, _epidx, _pas);
}

void UniqueAssignmentSuccessIterator::toNextValid()
{
    if (!_inSuccessRange) {
        while (_agentIdx < _pas->getTotalAgentCount() && _pas->getEntryPointIndexOf(_agentIdx) != _epIdx) {
            ++_agentIdx;
        }
        if (_agentIdx >= _pas->getTotalAgentCount()) {
            _agentIdx = 0;
            _inSuccessRange = true;
        }
    } else {
        const AgentGrp* successes = (_pas->getProblem()->getSuccess()->getAgentsByIndex(epIdx);
        if(successes) {
            while (_agentIdx < static_cast<int>(successes->size())) {
                AgentIDConstPtr id = *successes[_agentIdx];
                AssignmentIterator assignEnd{0, _pas->getTotalAgentCount(), _pas};
                auto it1 = std::find_if(AssignmentIterator(0, _epIdx, _pas), assignEnd, [id](AgentIDConstPtr a) { return *a == *id; });
                if (it1 == assignEnd) {
                    break;
                }
                auto it2 = std::find_if(successes->begin(), successes->begin() + _agentIdx, [id](AgentIDConstPtr a) { return *a == *id; });
                if (it2 == successes->begin() + _agentIdx) {
                    break;
                }
                ++_agentIdx;
            }
        }
    }
}

AssignmentView IAssignment::getRobotsWorking(const EntryPoint* ep) const
{
    return AssignmentView(ep->getIndex(), _impl);
}

AssignmentView IAssignment::getRobotsWorking(int64_t epid) const
{
    const EntryPointGrp& eps = _impl->getPlan()->getEntryPoints();
    for (int i = 0; i < static_cast<int>(eps.size()); ++i) {
        if (eps[i]->getId() == epid) {
            return AssignmentView(i, _impl);
        }
    }
    // return safe value that does not exist. Magic number is used for debuggers.
    return AssignmentView(-42, _impl);
}

AssignmentView IAssignment::getUnassignedAgents() const
{
    return AssignmentView(-1, _impl);
}
AssignmentSuccessView IAssignment::getRobotsWorkingAndFinished(const EntryPoint* ep) const
{
    return AssignmentSuccessView(ep->getIndex(), _impl);
}

AssignmentSuccessView IAssignment::getRobotsWorkingAndFinished(int64_t epid) const
{
    const EntryPointGrp& eps = _impl->getPlan()->getEntryPoints();
    for (int i = 0; i < static_cast<int>(eps.size()); ++i) {
        if (eps[i]->getId() == epid) {
            return AssignmentSuccessView(i, _impl);
        }
    }
    return AssignmentSuccessView(-42, _impl);
}

UniqueAssignmentSuccessView IAssignment::getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) const
{
    return UniqueAssignmentSuccessView(ep->getIndex(), _impl);
}
}