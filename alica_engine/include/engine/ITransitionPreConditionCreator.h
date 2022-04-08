#pragma once

#include <any>
#include <unordered_map>
#include <string>
#include <functional>

namespace alica
{
class BasicCondition;

class ITransitionPreConditionCreator
{
public:
    virtual ~ITransitionPreConditionCreator() {}
    virtual std::function<bool(std::unordered_map<std::string, std::any>)> createConditions(int64_t conditionId) = 0;
};

} /* namespace alica */
