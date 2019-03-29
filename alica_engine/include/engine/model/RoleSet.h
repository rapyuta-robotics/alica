#pragma once

#include <string>
#include <vector>

#include "AlicaElement.h"

namespace alica
{
class RoleTaskMapping;

class RoleSet : public AlicaElement
{
public:
    RoleSet();
    virtual ~RoleSet();
    std::string toString() const override;
    bool isDefault() const { return _isDefault; }
    const std::vector<RoleTaskMapping*>& getRoleTaskMappings() const { return _roleTaskMappings; }
    int64_t getUsableWithPlanId() const { return _usableWithPlanID; }

private:
    friend ModelFactory;
    void setRoleTaskMappings(const std::vector<RoleTaskMapping*>& roleTaskMappings);
    void setIsDefault(bool isDefault);
    void setUsableWithPlanId(int64_t usableWithPlanId);

    std::vector<RoleTaskMapping*> _roleTaskMappings;
    bool _isDefault;
    /**
     * the plan ID this roleset is defined for
     */
    int64_t _usableWithPlanID;
};

} // namespace alica
