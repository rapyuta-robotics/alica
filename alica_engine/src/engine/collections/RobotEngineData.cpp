#include "engine/collections/RobotEngineData.h"

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/collections/RobotProperties.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/model/AlicaElement.h"
#include "engine/model/DomainVariable.h"
#include "engine/model/ForallAgents.h"
#include "engine/model/Quantifier.h"
#include "engine/parser/PlanParser.h"

#include <supplementary/AgentID.h>

#include <assert.h>
#include <typeinfo>

namespace alica
{

/**
 * Basic constructor
 */
RobotEngineData::RobotEngineData(const AlicaEngine* engine, const supplementary::AgentID* agentId)
    : _engine(engine)
    , _agentId(agentId)
    , _successMarks(std::make_shared<SuccessMarks>(engine))
{
    initDomainVariables();
}

RobotEngineData::~RobotEngineData()
{
    for (auto x : _domainVariables) {
        delete x.second;
    }
}

void RobotEngineData::setSuccessMarks(shared_ptr<SuccessMarks> successMarks)
{
    _successMarks = successMarks;
}

void RobotEngineData::initDomainVariables()
{
    std::stringstream ss;
    ss << *_agentId << ".";
    std::string agentIdString = ss.str();
    for (const Quantifier* quantifier : _engine->getPlanRepository()->getQuantifiers()) {
        for (const Variable* tv : quantifier->getTemplateVariables()) {
            DomainVariable* dv = new DomainVariable(makeUniqueId(tv->getName()), agentIdString + tv->getName(), "", tv, _agentId);
            _domainVariables.emplace(tv, dv);
        }
    }
}

const DomainVariable* RobotEngineData::getDomainVariable(const Variable* templateVar) const
{
    auto iterator = _domainVariables.find(templateVar);
    if (iterator != _domainVariables.end()) {
        return iterator->second;
    } else {
        return nullptr;
    }
}

const DomainVariable* RobotEngineData::getDomainVariable(const std::string& sort) const
{
    const Variable* tv = _engine->getPlanRepository()->getVariables()[Hash64(sort.c_str(), sort.size())];
    return getDomainVariable(tv);
}

int64_t RobotEngineData::makeUniqueId(const std::string& s) const
{
    int64_t ret = (int64_t)(supplementary::AgentIDHash()(_agentId) + std::hash<string>()(s));
    assert(_engine->getPlanParser()->getParsedElements()->find(ret) == _engine->getPlanParser()->getParsedElements()->end());
    return ret;
}

void RobotEngineData::clearSuccessMarks()
{
    _successMarks->clear();
}

} /* namespace alica */
