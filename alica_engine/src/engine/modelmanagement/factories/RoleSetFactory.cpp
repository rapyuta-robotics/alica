#include "engine/modelmanagement/factories/RoleSetFactory.h"

#include "engine/model/RoleSet.h"
#include "engine/modelmanagement/factories/RoleFactory.h"

namespace alica
{
RoleSet* RoleSetFactory::create(const YAML::Node& node)
{
    auto* roleSet = new RoleSet(node);
    Factory::setAttributes(node, roleSet);
    Factory::storeElement(roleSet, alica::Strings::roleset);

    return roleSet;
}

void RoleSetFactory::attachReferences()
{
    RoleFactory::attachReferences();
}
} // namespace alica
