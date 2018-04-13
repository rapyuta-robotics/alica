/*
 * RoleTaskMapping.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLETASKMAPPING_H_
#define ROLETASKMAPPING_H_

#include <unordered_map>
#include <string>
#include <sstream>

#include "AlicaElement.h"

namespace alica {

class Role;
class ModelFactory;

class RoleTaskMapping : public AlicaElement {
public:
    RoleTaskMapping();
    virtual ~RoleTaskMapping();
    std::string toString() const override;
    const Role* getRole() const { return _role; }
    const std::unordered_map<int64_t, double>& getTaskPriorities() const { return _taskPriorities; }

private:
    friend ModelFactory;
    void setRole(const Role* role);
    void setTaskPriorities(const std::unordered_map<int64_t, double>& taskPriorities);
    const Role* _role;
    std::unordered_map<int64_t, double> _taskPriorities;
};

}  // namespace alica

#endif /* ROLETASKMAPPING_H_ */
