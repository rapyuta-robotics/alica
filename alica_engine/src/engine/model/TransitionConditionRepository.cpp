#include "engine/model/TransitionConditionRepository.h"

namespace alica
{

TransitionConditionRepository::TransitionConditionRepository() {}

void TransitionConditionRepository::addTransitionCondition(const TransitionCondition* t)
{
    _transitionConditions.push_back(t);
}

const std::string& TransitionConditionRepository::getFileName() const
{
    return _fileName;
}

void TransitionConditionRepository::setFileName(const std::string& fileName)
{
    _fileName = fileName;
}

} // namespace alica
