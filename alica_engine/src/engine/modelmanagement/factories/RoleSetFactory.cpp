#include "engine/modelmanagement/factories/RoleSetFactory.h"

#include "engine/modelmanagement/Strings.h"
#include "engine/model/Role.h"
#include "engine/model/RoleSet.h"
#include "engine/modelmanagement/factories/RoleFactory.h"

namespace alica
{
    RoleSet* RoleSetFactory::create(const YAML::Node& node)
    {
        RoleSet* roleSet = new RoleSet();
        Factory::setAttributes(node, roleSet);
        Factory::storeElement(roleSet, alica::Strings::roleset);

        if (Factory::isValid(node[alica::Strings::priorityDefault])) {
            roleSet->_priorityDefault = Factory::getValue<double>(node, alica::Strings::priorityDefault);
        }

        if (Factory::isValid(node[alica::Strings::roles])) {
            const YAML::Node &rolesNode = node[alica::Strings::roles];
            for (YAML::const_iterator it = rolesNode.begin(); it != rolesNode.end(); ++it) {
                roleSet->_roles.push_back(RoleFactory::create(*it, roleSet));
            }
        }

        return roleSet;
    }

    void RoleSetFactory::attachReferences() {
        RoleFactory::attachReferences();
    }
} // namespace alica