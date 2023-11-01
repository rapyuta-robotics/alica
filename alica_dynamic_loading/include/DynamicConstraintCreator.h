#pragma once
#include <engine/IConstraintCreator.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace alica
{

class BasicConstraint;

class DynamicConstraintCreator : public IConstraintCreator
{
public:
    DynamicConstraintCreator();
    virtual ~DynamicConstraintCreator(){};
    std::shared_ptr<BasicConstraint> createConstraint(int64_t conditionConfId, ConstraintContext& constraintContext) override;

private:
    typedef std::shared_ptr<BasicConstraint>(constraintCreatorType)(ConstraintContext&);
    std::unordered_map<std::string, std::function<constraintCreatorType>> _constraintCreatorMap; // See DynamicBehaviourCreator for an explanation
    std::vector<std::string> _libraryPath;
};

} /* namespace alica */
