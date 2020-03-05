#pragma once
#include <string>
#include <unordered_map>

#include "AlicaElement.h"

namespace alica
{
class ModelFactory;
class RoleFactory;
class RoleSet;
class Task;

class Role : public AlicaElement
{
public:
    Role();
    virtual ~Role();

    double getPriority(int64_t taskId) const;
    std::string toString(std::string indent = "") const override;

private:
    friend ModelFactory;
    friend RoleFactory;

    std::unordered_map<Task*, double> _taskPriorities;
    RoleSet* _roleSet;
};

} // namespace alica
