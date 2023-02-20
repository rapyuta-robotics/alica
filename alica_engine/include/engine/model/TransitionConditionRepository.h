#pragma once

#include "AlicaElement.h"
#include "engine/Types.h"

#include <string>

namespace alica
{
class ModelManager;
class TransitionConditionRepositoryFactory;

class TransitionConditionRepository : public AlicaElement
{
public:
    TransitionConditionRepository();
    virtual ~TransitionConditionRepository();
    const TransitionConditionGrp& getTransitionConditions() const { return _transitionConditions; }
    const std::string& getFileName() const;

private:
    friend ModelManager;
    friend TransitionConditionRepositoryFactory;
    void setFileName(const std::string& fileName);
    TransitionConditionGrp _transitionConditions;
    std::string _fileName;
};

} // namespace alica
