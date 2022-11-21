#pragma once

#include <engine/IConstraintCreator.h>
#include <memory>

namespace alica
{

class ConstraintCreator : public IConstraintCreator
{
public:
    ConstraintCreator();
    virtual ~ConstraintCreator();
    std::shared_ptr<BasicConstraint> createConstraint(int64_t constraintConfId) override;
};

} /* namespace alica */
