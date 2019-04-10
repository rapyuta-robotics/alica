#pragma once

#include "AlicaElement.h"

#include <string>
#include <vector>

namespace alica
{
class RoleTaskMapping;
class RoleSetFactory;
class Role;

class RoleSet : public AlicaElement
{
public:
    RoleSet();
    virtual ~RoleSet();
    std::string toString(std::string indent = "") const override;
    int64_t getUsableWithPlanId() const { return _usableWithPlanID; }
    const std::string& getFileName() const { return _fileName; }
    void setFileName(const std::string& fileName);
    const std::vector<Role*> getRoles() const { return _roles; }
    double getDefaultPriority() const { return _priorityDefault; }

private:
    friend ModelFactory;
    friend RoleSetFactory;
    void setUsableWithPlanId(int64_t usableWithPlanId);

    std::vector<Role*> _roles;
    /**
     * the default priority for all tasks that are not prioritized in a role of this set
     */
    double _priorityDefault;
    /**
     * the plan ID this roleset is defined for
     */
    int64_t _usableWithPlanID;
    /**
     * The file this abstract plan is parsed from/ written to.
     */
    std::string _fileName;
};

} // namespace alica
