#pragma once
#include <engine/IConstraintCreator.h>

#include <functional>
#include <memory>
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
    std::function<constraintCreatorType> _constraintCreator;
    std::vector<std::string> _libraryPath;
};

} /* namespace alica */
