#include "engine/planselector/IAssignment.h"

#include <engine/model/Plan.h>

namespace alica
{

PartialAssignmentSuccessIterator PartialAssignmentSuccessView::end() const
{
    const AgentGrp* agents = _pas->getProblem()->getSuccess(pas->getPlan())->getAgentsByIndex(_epidx);
    return PartialAssignmentSuccessIterator(agents ? agents->size() : 0, true, _epidx, _pas);
}

UniquePartialAssignmentSuccessIterator UniquePartialAssignmentSuccessView::end() const
{
    const AgentGrp* agents = _pas->getProblem()->getSuccess(pas->getPlan())->getAgentsByIndex(_epidx);
    return UniquePartialAssignmentSuccessIteraotr(agents ? agents->size() : 0, true, _epidx, _pas);
}

void UniquePartialAssignmentSuccessIterator::toNextValid()
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
                PartialAssignmentIterator assignEnd{0, _pas->getTotalAgentCount(), _pas};
                auto it1 = std::find_if(PartialAssignmentIterator(0, _epIdx, _pas), assignEnd, [id](AgentIDConstPtr a) { return *a == *id; });
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

PartialAssignmentView IAssignment::getRobotsWorking(const EntryPoint* ep) const
{
    return PartialAssignmentView(ep->getIndex(), _impl);
}

PartialAssignmentView IAssignment::getRobotsWorking(int64_t epid) const
{
    const EntryPointGrp& eps = _impl->getPlan()->getEntryPoints();
    for (int i = 0; i < static_cast<int>(eps.size()); ++i) {
        if (eps[i]->getId() == epid) {
            return PartialAssignmentView(i, _impl);
        }
    }
    // return safe value that does not exist. Magic number is used for debuggers.
    return PartialAssignmentView(-42, _impl);
}

PartialAssignmentView IAssignment::getUnassignedAgents() const
{
    return PartialAssignmentView(-1, _impl);
}
AssignmentSuccessView IAssignment::getRobotsWorkingAndFinished(const EntryPoint* ep) const
{
    return PartialAssignmentSuccessView(ep->getIndex(), _impl);
}

AssignmentSuccessView IAssignment::getRobotsWorkingAndFinished(int64_t epid) const
{
    const EntryPointGrp& eps = _impl->getPlan()->getEntryPoints();
    for (int i = 0; i < static_cast<int>(eps.size()); ++i) {
        if (eps[i]->getId() == epid) {
            return PartialAssignmentSuccessView(i, _impl);
        }
    }
    return PartialAssignmentSuccessView(-42, _impl);
}

UniquePartialAssignmentSuccessView IAssignment::getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) const
{
    return UniquePartialAssignmentSuccessView(ep->getIndex(), _impl);
}
}