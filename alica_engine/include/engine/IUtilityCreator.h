#pragma once

#include <memory>

namespace alica
{
// DO NOT REMOVE UNTIL templates can be reworked
using std::shared_ptr;

class BasicUtilityFunction;

class IUtilityCreator
{
public:
    virtual ~IUtilityCreator() {}

    virtual std::shared_ptr<BasicUtilityFunction> createUtility(long utilityfunctionConfId) = 0;
};

} /* namespace alica */
