#pragma once
#include <string>
#include <unordered_map>

#include "AlicaElement.h"

namespace alica
{
class RoleFactory;
class RoleSet;
class Task;

class Role : public AlicaElement
{
public:
    Role(RoleSet* roleSet);

    double getPriority(int64_t taskId) const;
    void setPriority(Task* task, double priority);
    std::string toString(std::string indent = "") const override;

private:
    std::unordered_map<Task*, double> _taskPriorities;
    RoleSet* _roleSet;
};

} // namespace alica
