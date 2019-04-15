#pragma once

#include <memory>
#include <string>

#include "AlicaElement.h"
#include "engine/AlicaClock.h"

#include "engine/Types.h"

namespace alica
{

class Variable;
class UtilityFunction;
class ModelFactory;
class ExpressionHandler;
class ModelManager;
class AbstractPlanFactory;

/**
 * Super class of plans, plantypes and behaviours.
 */
class AbstractPlan : public AlicaElement
{
public:
    AbstractPlan();
    AbstractPlan(int64_t id);

    virtual ~AbstractPlan();

    bool containsVar(const Variable* v) const;
    bool containsVar(const std::string& name) const;
    const VariableGrp& getVariables() const { return _variables; }
    const Variable* getVariable(const std::string& name) const;

    AlicaTime getAuthorityTimeInterval() const { return _authorityTimeInterval; }
    void setAuthorityTimeInterval(AlicaTime authorityTimeInterval) const; // not a mistake, this is mutable

    std::string toString(std::string indent = "") const override;
    const std::string& getFileName() const { return _fileName; }

private:
    friend ModelFactory;
    friend ModelManager;
    friend AbstractPlanFactory;
    friend ExpressionHandler;

    void setFileName(const std::string& fileName);
    /**
     * The variables that are available in the context of this abstract plan.
     */
    VariableGrp _variables;

    /**
     * The file this abstract plan is parsed from/ written to.
     */
    std::string _fileName;

    // TODO: move this to the authority module
    mutable AlicaTime _authorityTimeInterval;
};

} // namespace alica
