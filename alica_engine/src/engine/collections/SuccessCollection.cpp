#include "engine/collections/SuccessCollection.h"

#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/Task.h"

#include <algorithm>
#include <sstream>

namespace alica
{
SuccessCollection::SuccessCollection()
        : _plan(nullptr)
        , _successData()
{
}

SuccessCollection::SuccessCollection(std::size_t parentContextHash, const Plan* plan)
        : _parentContextHash(parentContextHash)
        , _plan(plan)
        , _successData(plan->getEntryPoints().size())
{
}

SuccessCollection::~SuccessCollection() {}

void SuccessCollection::setSuccess(AgentId agentId, const EntryPoint* ep)
{
    if (ep->getPlan() == _plan) {
        _successData[ep->getIndex()].push_back(agentId);
    }
}

void SuccessCollection::clear()
{
    for (AgentGrp& ag : _successData) {
        ag.clear();
    }
}

const AgentGrp* SuccessCollection::getAgents(const EntryPoint* ep) const
{
    if (ep->getPlan() == _plan) {
        return &_successData[ep->getIndex()];
    }
    return nullptr;
}

const AgentGrp* SuccessCollection::getAgentsById(int64_t id) const
{
    const EntryPointGrp& eps = _plan->getEntryPoints();
    for (int i = 0; i < static_cast<int>(eps.size()); ++i) {
        if (eps[i]->getId() == id) {
            return &_successData[i];
        }
    }
    return nullptr;
}

std::ostream& operator<<(std::ostream& out, const SuccessCollection& c)
{
    bool haveAny = false;
    const EntryPointGrp& eps = c._plan->getEntryPoints();
    for (int i = 0; i < static_cast<int>(eps.size()); ++i) {
        if (!c._successData[i].empty()) {
            if (!haveAny) {
                out << "Success:" << std::endl;
            }
            haveAny = true;
            out << "id: " << eps[i]->getId() << ", dynamic_id: " << eps[i]->getDynamicId() << " (" << eps[i]->getTask()->getName() << "): ";
            for (AgentId r : c._successData[i]) {
                out << r << " ";
            }
            out << std::endl;
        }
    }
    if (!haveAny) {
        out << "No EntryPoint successful.";
    }
    return out;
}

} /* namespace alica */
