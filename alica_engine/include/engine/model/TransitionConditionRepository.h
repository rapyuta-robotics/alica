#pragma once

#include "AlicaElement.h"
#include "engine/Types.h"

#include <string>

namespace alica
{
class TransitionConditionRepository : public AlicaElement
{
public:
    TransitionConditionRepository();
    void addTransitionCondition(const TransitionCondition* t);
    const TransitionConditionGrp& getTransitionConditions() const { return _transitionConditions; }
    void setFileName(const std::string& fileName);
    const std::string& getFileName() const;

private:
    TransitionConditionGrp _transitionConditions;
    std::string _fileName;
};

} // namespace alica
