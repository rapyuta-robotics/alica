#include "engine/model/RoleSet.h"
#include "engine/model/Role.h"

#include <sstream>

namespace alica
{

RoleSet::RoleSet()
        : _usableWithPlanID(0)
        , _priorityDefault(0)
{
}

RoleSet::~RoleSet() {}

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
