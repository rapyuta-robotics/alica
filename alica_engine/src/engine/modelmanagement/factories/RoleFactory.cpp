#include "engine/modelmanagement/factories/RoleFactory.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/model/Role.h"

namespace alica
{
    Role* RoleFactory::create(const YAML::Node& roleNode, RoleSet* roleSet)
    {
        Role* role = new Role();
        Factory::setAttributes(roleNode, role);
        Factory::storeElement(role, alica::Strings::role);
        role->_roleSet = roleSet;

        if (Factory::isValid(roleNode[alica::Strings::taskPriorities])) {
            const YAML::Node& taskPriorities = roleNode[alica::Strings::taskPriorities];
            for (YAML::const_iterator it = taskPriorities.begin(); it != taskPriorities.end(); ++it) {
                // TODO Key and Value
                //Factory::getReferencedId(*it)));
            }
        }

        return role;
    }
} // namespace alica
