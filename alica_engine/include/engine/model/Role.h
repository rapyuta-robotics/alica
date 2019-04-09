#pragma once
#include <string>
#include <unordered_map>

#include "AlicaElement.h"

namespace alica
{
class ModelFactory;
class RoleFactory;
class RoleSet;

class Role : public AlicaElement
{
public:
    Role();
    virtual ~Role();

    double getPriority(int64_t taskId) const;
    std::string toString() const override;

    const std::unordered_map<int64_t, double>& getTaskPriorities() const { return _taskPriorities; }
    void setTaskPriorities(const std::unordered_map<int64_t, double>& taskPriorities);

private:
    friend ModelFactory;
    friend RoleFactory;

    std::unordered_map<int64_t, double> _taskPriorities;
    RoleSet* _roleSet;
};

} // namespace alica
