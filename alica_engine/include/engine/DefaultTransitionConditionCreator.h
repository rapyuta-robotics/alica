#pragma once

#include "engine/Types.h"
#include <string>
#include <iostream>

namespace alica
{

class DefaultTransitionConditionCreator
{
public:
    DefaultTransitionConditionCreator() = default;
    ~DefaultTransitionConditionCreator() = default;
    TransitionConditionCallback createConditions(std::string name) const;
    bool isDefaultTransitionCondition(std::string name) const;
    // TransitionConditionCallback createConditions(std::string name) const
    // {
    //     if (name == "DefaultCondition") {
    //         return std::bind(defaultCondition, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    //     }
    //     std::cerr << "DefaultTransitionConditionCreator: Unknown condition requested: " << name << std::endl;
    //     throw new std::exception();
    // }

    // bool isDefaultTransitionCondition(std::string name) const
    // {
    //     if (name == "DefaultCondition") {
    //         return true;
    //     }
    //     return false;
    // }
};

} /* namespace alica */
