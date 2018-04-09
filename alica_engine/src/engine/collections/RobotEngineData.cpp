#include "engine/collections/RobotEngineData.h"

#include "engine/AlicaEngine.h"
#include "engine/parser/PlanParser.h"
#include "engine/PlanRepository.h"
#include "engine/collections/RobotProperties.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/model/AlicaElement.h"
#include "engine/model/ForallAgents.h"
#include "engine/model/Quantifier.h"
#include "engine/model/Variable.h"

#include <supplementary/AgentID.h>
#include <typeinfo>
namespace alica {

/**
 * Basic constructor
 */
RobotEngineData::RobotEngineData(const AlicaEngine* engine, const supplementary::AgentID* agentId)
        : engine(engine)
        , agentId(agentId) {
    this->initDomainVariables();
    this->successMarks = make_shared<SuccessMarks>(engine);
}

RobotEngineData::~RobotEngineData() {
    for (auto x : this->domainVariables) {
        delete x.second;
    }
}

shared_ptr<SuccessMarks> RobotEngineData::getSuccessMarks() const {
    return successMarks;
}

void RobotEngineData::setSuccessMarks(shared_ptr<SuccessMarks> successMarks) {
    this->successMarks = successMarks;
}

void RobotEngineData::initDomainVariables() {
    stringstream ss;
    ss << this->agentId << ".";
    string agentIdString = ss.str();
    for (const Quantifier* quantifier : engine->getPlanRepository()->getQuantifiers()) {
        if (dynamic_cast<const ForallAgents*>(quantifier) != nullptr) {
            for (const std::string& s : quantifier->getDomainIdentifiers()) {
                Variable* v = new Variable(makeUniqueId(s), agentIdString + s, "");
                this->domainVariables.insert(std::pair<std::string, const Variable*>(s, v));
            }
        }
    }
}

const Variable* RobotEngineData::getDomainVariable(const std::string& sort) const {
    auto iterator = this->domainVariables.find(sort);
    if (iterator != this->domainVariables.end()) {
        return iterator->second;
    } else {
        return nullptr;
    }
}

int64_t RobotEngineData::makeUniqueId(const std::string& s) const {
    int64_t ret = (int64_t) (supplementary::AgentIDHash()(this->agentId) + std::hash<string>()(s));
    if (this->engine->getPlanParser()->getParsedElements()->find(ret) !=
            this->engine->getPlanParser()->getParsedElements()->end()) {
        this->engine->abort("TO: Hash Collision in generating a quantified variable's unique ID: ", ret);
    }
    return ret;
}

void RobotEngineData::clearSuccessMarks() {
    this->successMarks->clear();
}

} /* namespace alica */
