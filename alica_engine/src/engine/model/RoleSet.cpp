#include "engine/model/RoleSet.h"
#include "engine/model/Role.h"
#include "engine/modelmanagement/Strings.h"

#include <sstream>

namespace alica
{

RoleSet::RoleSet(const YAML::Node& node)
        : _usableWithPlanID(0)
        , _priorityDefault(0)
{
    if (Factory::isValid(node[alica::Strings::priorityDefault])) {
        _priorityDefault = Factory::getValue<double>(node, alica::Strings::priorityDefault);
    }

    if (Factory::isValid(node[alica::Strings::roles])) {
        const YAML::Node& rolesNode = node[alica::Strings::roles];
        for (YAML::const_iterator it = rolesNode.begin(); it != rolesNode.end(); ++it) {
            _roles.push_back(RoleFactory::create(*it, this));
        }
    }
}

std::string RoleSet::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "#RoleSet: " << getName() << " " << getId() << std::endl;
    ss << indent << "\t UsableWithPlanID: " << _usableWithPlanID << std::endl;
    ss << indent << "\t Contains Mappings: " << _roles.size() << std::endl;
    for (const Role* role : _roles) {
        ss << role->toString(indent + "\t");
    }
    ss << "#EndRoleSet" << std::endl;
    return ss.str();
}

void RoleSet::setUsableWithPlanId(int64_t usableWithPlanId)
{
    _usableWithPlanID = usableWithPlanId;
}

void RoleSet::setFileName(const std::string& fileName)
{
    _fileName = fileName;
}

} // namespace alica
