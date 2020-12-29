#include "engine/model/AbstractPlan.h"
#include "engine/model/Variable.h"
#include "engine/AlicaEngine.h"

#include <yaml-cpp/yaml.h>
#include <sstream>
#include <functional>

namespace alica
{

AbstractPlan::AbstractPlan(AlicaEngine *ae)
        : AlicaElement()

{
    auto reloadFunctionPtr = std::bind(&AbstractPlan::reload, this, std::placeholders::_1);
    ae->subscribe(reloadFunctionPtr);
    reload(ae->getConfig());
}

AbstractPlan::AbstractPlan(AlicaEngine *ae, int64_t id)
        : AlicaElement(id)
{
    auto reloadFunctionPtr = std::bind(&AbstractPlan::reload, this, std::placeholders::_1);
    ae->subscribe(reloadFunctionPtr);
    reload(ae->getConfig());
}

AbstractPlan::~AbstractPlan() {}

void AbstractPlan::reload(const YAML::Node& config)
{
    _authorityTimeInterval = AlicaTime::milliseconds(config["Alica"]["CycleDetection"]["MinimalAuthorityTimeInterval"].as<unsigned long>());
}

std::string AbstractPlan::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << AlicaElement::toString(indent);
    ss << indent << "\tFilename: " << _fileName << std::endl;
    return ss.str();
}

const Variable* AbstractPlan::getVariable(const std::string& name) const
{
    for (const Variable* variable : _variables) {
        if (variable->getName() == name) {
            return variable;
        }
    }
    return nullptr;
}
/**
 * Tests whether a given variable belongs to this plan.
 * @param v A Variable*
 * @return A bool
 */
bool AbstractPlan::containsVar(const Variable* v) const
{
    return find(_variables.begin(), _variables.end(), v) != _variables.end();
}

bool AbstractPlan::containsVar(const std::string& name) const
{
    for (const Variable* v : _variables) {
        if (v->getName() == name) {
            return true;
        }
    }
    return false;
}

void AbstractPlan::setAuthorityTimeInterval(AlicaTime authorithyTimeInterval) const
{
    _authorityTimeInterval = authorithyTimeInterval;
}

void AbstractPlan::setFileName(const std::string& fileName)
{
    _fileName = fileName;
}

} // namespace alica
