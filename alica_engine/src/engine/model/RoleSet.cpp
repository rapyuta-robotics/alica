#include "engine/model/RoleSet.h"
#include <sstream>

namespace alica
{

RoleSet::RoleSet()
        : _usableWithPlanID(0)
{
}

RoleSet::~RoleSet() {}

std::string RoleSet::toString() const
{
    std::stringstream ss;
    ss << "#RoleSet: " << getName() << " " << getId() << std::endl;
    ss << "\t UsableWithPlanID: " << _usableWithPlanID << std::endl;
    ss << "\t Contains Mappings: " << _roles.size() << std::endl;
    for (const Role* role : _roles) {
        ss << "\tRole: " << role << std::endl;
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
