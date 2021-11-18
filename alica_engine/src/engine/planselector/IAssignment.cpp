#include "engine/planselector/IAssignment.h"
#include "engine/planselector/TaskAssignmentProblem.h"

#include <engine/model/Plan.h>

namespace alica
{

PartialAssignmentSuccessIterator PartialAssignmentSuccessView::end() const
{
    const AgentGrp* agents = _pas->getSuccessData()->getAgentsByIndex(_epIdx);
    return PartialAssignmentSuccessIterator(agents ? agents->size() : 0, true, _epIdx, _pas);
}

UniquePartialAssignmentSuccessIterator UniquePartialAssignmentSuccessView::end() const
{
    const AgentGrp* agents = _pas->getSuccessData()->getAgentsByIndex(_epIdx);
    return UniquePartialAssignmentSuccessIterator(agents ? agents->size() : 0, true, _epIdx, _pas);
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
        const AgentGrp* successes = _pas->getSuccessData()->getAgentsByIndex(_epIdx);
        if (successes) {
            while (_agentIdx < static_cast<int>(successes->size())) {
                AgentId id = (*successes)[_agentIdx];
                PartialAssignmentIterator assignEnd{_pas->getTotalAgentCount(), _epIdx, _pas};
                auto it1 = std::find(PartialAssignmentIterator(0, _epIdx, _pas), assignEnd, id);
                if (it1 == assignEnd) {
                    break;
                }

                auto it2 = std::find(successes->begin(), successes->begin() + _agentIdx, id);
                if (it2 == successes->begin() + _agentIdx) {
                    break;
                }
                ++_agentIdx;
            }
        }
    }
}

PartialAssignmentView IAssignment::getAgentsWorking(const EntryPoint* ep) const
{
    return PartialAssignmentView(ep->getIndex(), _impl);
}

PartialAssignmentView IAssignment::getAgentsWorking(int64_t epid) const
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
PartialAssignmentSuccessView IAssignment::getAgentsWorkingAndFinished(const EntryPoint* ep) const
{
    return PartialAssignmentSuccessView(ep->getIndex(), _impl);
}

PartialAssignmentSuccessView IAssignment::getAgentsWorkingAndFinished(int64_t epid) const
{
    const EntryPointGrp& eps = _impl->getPlan()->getEntryPoints();
    for (int i = 0; i < static_cast<int>(eps.size()); ++i) {
        if (eps[i]->getId() == epid) {
            return PartialAssignmentSuccessView(i, _impl);
        }
    }
    return PartialAssignmentSuccessView(-42, _impl);
}

UniquePartialAssignmentSuccessView IAssignment::getUniqueAgentsWorkingAndFinished(const EntryPoint* ep) const
{
    return UniquePartialAssignmentSuccessView(ep->getIndex(), _impl);
}
} // namespace alica