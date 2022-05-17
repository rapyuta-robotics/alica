#include "engine/model/ConditionRepository.h"

namespace alica
{

ConditionRepository::ConditionRepository() {}

ConditionRepository::~ConditionRepository() {}

std::string ConditionRepository::getFileName() const
{
    return _fileName;
}

void ConditionRepository::setFileName(const std::string& fileName)
{
    _fileName = fileName;
}

} // namespace alica
