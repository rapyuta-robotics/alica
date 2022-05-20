#include "engine/model/TransitionConditionRepository.h"

namespace alica
{

TransitionConditionRepository::TransitionConditionRepository() {}

TransitionConditionRepository::~TransitionConditionRepository() {}

std::string TransitionConditionRepository::getFileName() const
{
    return _fileName;
}

void TransitionConditionRepository::setFileName(const std::string& fileName)
{
    _fileName = fileName;
}

} // namespace alica
