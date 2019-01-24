#include "engine/collections/RobotEngineData.h"

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/collections/RobotProperties.h"
#include "engine/model/AlicaElement.h"
#include "engine/model/DomainVariable.h"
#include "engine/model/ForallAgents.h"
#include "engine/model/Quantifier.h"
#include "engine/parser/PlanParser.h"

#include <assert.h>
#include <typeinfo>

namespace alica
{

/**
 * Basic constructor
 */
RobotEngineData::RobotEngineData(AlicaEngine* engine, AgentIDConstPtr agentId)
        : _engine(engine)
        , _agentId(agentId)
        , _successMarks()
{
    initDomainVariables();
}

RobotEngineData::~RobotEngineData() {}

void RobotEngineData::updateSuccessMarks(const IdGrp& succeededEps)
{
    _successMarks.update(_engine, succeededEps);
}

void RobotEngineData::initDomainVariables()
{
    std::stringstream ss;
    ss << *_agentId << ".";
    std::string agentIdString = ss.str();
    for (const Quantifier* quantifier : _engine->getPlanRepository().getQuantifiers()) {
        for (const Variable* tv : quantifier->getTemplateVariables()) {
            _domainVariables.emplace(tv,
                    std::unique_ptr<const DomainVariable>(new DomainVariable(makeUniqueId(tv->getName()), agentIdString + tv->getName(), "", tv, _agentId)));
        }
    }
}

const DomainVariable* RobotEngineData::getDomainVariable(const Variable* templateVar) const
{
    auto iterator = _domainVariables.find(templateVar);
    if (iterator != _domainVariables.end()) {
        return iterator->second.get();
    } else {
        return nullptr;
    }
}

const DomainVariable* RobotEngineData::getDomainVariable(const std::string& name) const
{
    const Variable* tv = _engine->getPlanRepository().getVariables()[Hash64(name.c_str(), name.size())];
    return getDomainVariable(tv);
}

int64_t RobotEngineData::makeUniqueId(const std::string& s) const
{
    int64_t ret = static_cast<int64_t>(essentials::AgentIDHash{}(_agentId.get()) + std::hash<std::string>()(s));
    assert(_engine->getPlanParser().isUniqueElement(ret));
    return ret;
}

void RobotEngineData::clearSuccessMarks()
{
    _successMarks.clear();
}

} /* namespace alica */
