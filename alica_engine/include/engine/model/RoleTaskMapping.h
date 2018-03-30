/*
 * RoleTaskMapping.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLETASKMAPPING_H_
#define ROLETASKMAPPING_H_

#include <map>
#include <string>
#include <sstream>

#include "AlicaElement.h"

using namespace std;
namespace alica {

class Role;

class RoleTaskMapping : public AlicaElement {
public:
    RoleTaskMapping();
    virtual ~RoleTaskMapping();

    string toString();

    const Role* getRole() const;
    void setRole(const Role* role);
    map<long, double>& getTaskPriorities();
    void setTaskPriorities(const map<long, double>& taskPriorities);

protected:
    const Role* role;
    map<long, double> taskPriorities;
};

}  // namespace alica

#endif /* ROLETASKMAPPING_H_ */
