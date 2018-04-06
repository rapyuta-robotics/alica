/*
 * Role.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLE_H_
#define ROLE_H_

#include <unordered_map>
#include <string>

#include "AlicaElement.h"


namespace alica {
class RoleTaskMapping;
class RoleDefinitionSet;
class Characteristic;
class ModelFactory;

class Role : public AlicaElement {
public:
    Role();
    virtual ~Role();

    double getPriority(int64_t taskId) const;
    std::string toString() const;

    const std::unordered_map<std::string, const Characteristic*>& getCharacteristics() const {return _characteristics;}
    const RoleDefinitionSet* getRoleDefinitionSet() const {return _roleDefinitionSet;}
    const RoleTaskMapping* getRoleTaskMapping() const {return _roleTaskMapping;}

private:
    friend ModelFactory;
    void setRoleDefinitionSet(const RoleDefinitionSet* roleDefinitionSet);
    void setRoleTaskMapping(const RoleTaskMapping* roleTaskMapping);

    std::unordered_map<std::string, const Characteristic*> _characteristics;
    const RoleTaskMapping* _roleTaskMapping;
    const RoleDefinitionSet* _roleDefinitionSet;
};

}  // namespace alica

#endif /* ROLE_H_ */
