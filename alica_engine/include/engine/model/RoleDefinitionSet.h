/*
 * RoleDefinitionSet.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLEDEFINITIONSET_H_
#define ROLEDEFINITIONSET_H_

#include <string>

#include "AlicaElement.h"
#include "engine/Types.h"

namespace alica {
class ModelFactory;

class RoleDefinitionSet : public AlicaElement {
public:
    RoleDefinitionSet();
    virtual ~RoleDefinitionSet();
    const std::string& getFileName() const { return _fileName; }
    const RoleGrptor& getRoles() const { return _roles; }

private:
    friend ModelFactory;
    void setRoles(const RoleGrptor& roles);
    void setFileName(const std::string& fileName);

    RoleGrptor _roles;
    std::string _fileName;
};

}  // namespace alica

#endif /* ROLEDEFINITIONSET_H_ */
