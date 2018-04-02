/*
 * Role.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLE_H_
#define ROLE_H_

#include <map>
#include <string>
#include <sstream>
#include <exception>
#include <iostream>

#include "AlicaElement.h"

using namespace std;
namespace alica {
class RoleTaskMapping;
class RoleDefinitionSet;
class Characteristic;

class Role : public AlicaElement {
public:
    Role();
    virtual ~Role();

    double getPriority(long taskId);
    string toString();

    map<string, Characteristic*>& getCharacteristics();
    const RoleDefinitionSet* getRoleDefinitionSet() const;
    void setRoleDefinitionSet(const RoleDefinitionSet* roleDefinitionSet);
    const RoleTaskMapping* getRoleTaskMapping() const;
    void setRoleTaskMapping(RoleTaskMapping* roleTaskMapping);

protected:
    RoleTaskMapping* roleTaskMapping;
    map<string, Characteristic*> characteristics;
    const RoleDefinitionSet* roleDefinitionSet;
};

}  // namespace alica

#endif /* ROLE_H_ */
