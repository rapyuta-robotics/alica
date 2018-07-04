/*
 * AbstractPlan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ABSTRACTPLAN_H_
#define ABSTRACTPLAN_H_

#include <memory>
#include <string>

#include "AlicaElement.h"
#include "engine/AlicaClock.h"

#include "engine/Types.h"

namespace alica
{

class Variable;
class PreCondition;
class RuntimeCondition;
class UtilityFunction;
class ModelFactory;
class ExpressionHandler;

/**
 * Super class of plans, plantypes and behaviourconfigurations.
 */
class AbstractPlan : public AlicaElement
{
public:
    AbstractPlan();
    AbstractPlan(int64_t id);

    virtual ~AbstractPlan();

    bool containsVar(const Variable* v) const;
    bool containsVar(const std::string& name) const;

    bool isMasterPlan() const { return _masterPlan; }
    AlicaTime getAuthorityTimeInterval() const { return _authorityTimeInterval; }
    const VariableGrp& getVariables() const { return _variables; }
    const RuntimeCondition* getRuntimeCondition() const { return _runtimeCondition; }
    const PreCondition* getPreCondition() const { return _preCondition; }

    std::string toString() const override;
    const std::string& getFileName() const { return _fileName; }

    void setAuthorityTimeInterval(AlicaTime authorityTimeInterval) const; // not a mistake, this is mutable
    const Variable* getVariableByName(const std::string& name) const;

private:
    friend ModelFactory;
    friend ExpressionHandler;

    void setMasterPlan(bool isMasterPlan);

    void setFileName(const std::string& fileName);
    void setVariables(const VariableGrp& variables);
    void setRuntimeCondition(RuntimeCondition* runtimeCondition);
    void setPreCondition(PreCondition* preCondition);

    // TODO: move this to the authority module
    mutable AlicaTime _authorityTimeInterval;
    /**
     * This plan's runtime condition.
     */
    RuntimeCondition* _runtimeCondition;
    /**
     * This plan's precondition
     */
    PreCondition* _preCondition;

    VariableGrp _variables;

    /**
     *  Whether this plan is marked as a MasterPlan.
     */
    bool _masterPlan;
    std::string _fileName;
};

} // namespace alica

#endif /* ABSTRACTPLAN_H_ */
