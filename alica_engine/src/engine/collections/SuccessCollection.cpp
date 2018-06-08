
#include <engine/collections/SuccessCollection.h>

#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/model/Task.h"

#include <algorithm>
#include <sstream>

namespace alica
{

SuccessCollection::SuccessCollection(const Plan* plan)
        : _plan(plan)
        , _successData(plan->getEntryPoints().size())
{
}

SuccessCollection::~SuccessCollection() {}

void SuccessCollection::setSuccess(AgentIDConstPtr robotId, const EntryPoint* ep)
{
    for (int i = 0; i < static_cast<int>(_plan->getEntryPoints().size()); ++i) {
        if _plan->getEntryPoints()[i] == ep)
            {
                _successData[i].push_back(robotId);
                return;
            }
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
    const EntryPointGrp& eps = _plan->getEntryPoints();
    for (int i = 0; i < static_cast<int>(eps.size()); ++i) {
        if (eps[i] == ep) {
            return &_successData[i];
        }
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
    const EntryPointGrp& eps = _plan->getEntryPoints();
    for (int i = 0; i < static_cast<int>(eps.size()); ++i) {
        if (!_successData[i].empty()) {
            if (!haveAny) {
                out << "Success:" << std::endl;
            }
            haveAny = true;
            out << eops[i]->getTask()->getId() << ": ";
            for (AgentIDConstPtr r : _successData[i]) {
                out << *r << " ";
            }
            out << std::endl;
        }
    }
    if (!haveAny) {
        out << "No EntryPoint successful!";
    }
    return out;
}

} /* namespace alica */
