/*
 * AbstractPlan.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ABSTRACTPLAN_H_
#define ABSTRACTPLAN_H_


#include <string>
#include <memory>

#include "AlicaElement.h"
#include "engine/IAlicaClock.h"

#include "engine/Types.h"

namespace alica {

class Variable;
class PreCondition;
class RuntimeCondition;
class UtilityFunction;
class ModelFactory;
class ExpressionHandler;

/**
 * Super class of plans, plantypes and behaviourconfigurations.
 */
class AbstractPlan : public AlicaElement {
public:
    AbstractPlan();
    AbstractPlan(int64_t id);

    virtual ~AbstractPlan();

    bool containsVar(const Variable* v) const;
    bool containsVar(const std::string& name) const;

    bool isMasterPlan() const {return _masterPlan;}
    AlicaTime getAuthorityTimeInterval() const {return _authorityTimeInterval;}
    const VariableSet& getVariables() const {return _variables;}
    const RuntimeCondition* getRuntimeCondition() const {return _runtimeCondition;}
    const PreCondition* getPreCondition() const {return _preCondition;}
    const std::shared_ptr<UtilityFunction>& getUtilityFunction() const {return _utilityFunction;}
    double getUtilityThreshold() const {return _utilityThreshold;}

    std::string toString() const;
    const std::string& getFileName() const {return _fileName;}
    
    void setAuthorityTimeInterval(AlicaTime authorityTimeInterval) const; //not a mistake, this is mutable

private:
    friend ModelFactory;
    friend ExpressionHandler;

    void setMasterPlan(bool isMasterPlan);
    
    void setFileName(const std::string& fileName);
    void setVariables(const VariableSet& variables);
    void setRuntimeCondition(RuntimeCondition* runtimeCondition);
    void setPreCondition(PreCondition* preCondition);
    void setUtilityFunction(std::shared_ptr<UtilityFunction> utilityFunction);
    void setUtilityThreshold(double utilityThreshold);

    //TODO: move this to the authority module
    mutable AlicaTime _authorityTimeInterval;
    /**
     * This plan's runtime condition.
     */
    RuntimeCondition* _runtimeCondition;
    /**
     * This plan's precondition
     */
    PreCondition* _preCondition;
    /**
     * This plan's Utility function
     */
    std::shared_ptr<UtilityFunction> _utilityFunction; //TODO why the heck is this a shared ptr, livetime is bout to this object

    
    VariableSet _variables;
    /**
     * The utility threshold, the higher, the less likely dynamic changes are.
     */
    double _utilityThreshold;

    /**
     *  Whether this plan is marked as a MasterPlan.
     */
    bool _masterPlan;
    std::string _fileName;

};

}  // namespace alica

#endif /* ABSTRACTPLAN_H_ */
