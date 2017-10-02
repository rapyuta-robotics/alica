#include "engine/collections/RobotEngineData.h"

#include "engine/AlicaEngine.h"
#include "engine/IPlanParser.h"
#include "supplementary/IAgentID.h"
#include "engine/PlanRepository.h"
#include "engine/collections/RobotProperties.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/model/AlicaElement.h"
#include "engine/model/ForallAgents.h"
#include "engine/model/Quantifier.h"
#include "engine/model/Variable.h"

namespace alica
{

/**
 * Basic constructor
 */
RobotEngineData::RobotEngineData(const AlicaEngine *engine, const supplementary::IAgentID *agentId)
    : engine(engine), agentId(agentId)
{
    this->initDomainVariables();
    this->successMarks = make_shared<SuccessMarks>(engine);
}

RobotEngineData::~RobotEngineData()
{
    for (auto x : this->domainVariables)
    {
        delete x.second;
    }
}

shared_ptr<SuccessMarks> RobotEngineData::getSuccessMarks()
{
    return successMarks;
}

void RobotEngineData::setSuccessMarks(shared_ptr<SuccessMarks> successMarks)
{
    this->successMarks = successMarks;
}

void RobotEngineData::initDomainVariables()
{
    auto qs = this->engine->getPlanRepository()->getQuantifiers();
    stringstream ss;
    ss << this->agentId << ".";
    string agentIdString = ss.str();
    for (auto quantifierPair : qs)
    {
        if (dynamic_cast<ForallAgents *>(quantifierPair.second) != nullptr)
        {
            for (string s : quantifierPair.second->getDomainIdentifiers())
            {
                Variable *v = new Variable(makeUniqueId(s), agentIdString + s, "");
                this->domainVariables.insert(pair<string, Variable *>(s, v));
            }
        }
    }
}

Variable *RobotEngineData::getDomainVariable(string sort) const
{
    auto iterator = this->domainVariables.find(sort);
    if (iterator != this->domainVariables.end())
    {
        return iterator->second;
    }
    else
    {
        return nullptr;
    }
}

long RobotEngineData::makeUniqueId(string s)
{
    long ret = (long)(std::hash<supplementary::IAgentID>()(*this->agentId) + std::hash<string>()(s));
    if (this->engine->getPlanParser()->getParsedElements()->find(ret) !=
        this->engine->getPlanParser()->getParsedElements()->end())
    {
        this->engine->abort("TO: Hash Collision in generating a quantified variable's unique ID: ", ret);
    }
    return ret;
}

void RobotEngineData::clearSuccessMarks()
{
	this->successMarks->clear();
}

} /* namespace alica */
