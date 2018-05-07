#pragma once

#include <engine/model/Variable.h>

namespace alica
{

class DomainVariable : public Variable
{
  public:
    DomainVariable(int64_t id, const std::string& name, const std::string& type, const Variable* templateVar, AgentIDConstPtr agent)
        : Variable(id, name, type)
        , _templateVar(templateVar)
        , _agentId(agent)
    {
    }
    const Variable* getTemplateVariable() const { return _templateVar; }
    AgentIDConstPtr getAgent() const { return _agentId; }

  private:
    const Variable* _templateVar;
    AgentIDConstPtr _agentId;
};
}