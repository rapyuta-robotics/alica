#pragma once

#include "AlicaElement.h"
#include "engine/Types.h"

#include <string>

namespace alica
{
class ModelManager;
class ConditionRepositoryFactory;

class ConditionRepository : public AlicaElement
{
public:
    ConditionRepository();
    virtual ~ConditionRepository();
    const ConditionGrp& getConditions() const { return _conditions; }
    std::string getFileName() const;

private:
    friend ModelFactory;
    friend ModelManager;
    friend ConditionRepositoryFactory;
    void setFileName(const std::string& fileName);
    ConditionGrp _conditions;
    std::string _fileName;
};

} // namespace alica
