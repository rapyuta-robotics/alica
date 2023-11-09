#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>
#include <engine/PlanStatus.h>

namespace alica
{
// // used for accessing bb values with an unknown type
// class UnknownType
// {
// public:
//     UnknownType() = default;
//     UnknownType(int64_t input)
//             : value(input){};
//     int64_t value;
// };

class OverwriteValueWithDifferentTypeTestBeh : public BasicBehaviour
{
public:
    OverwriteValueWithDifferentTypeTestBeh(BehaviourContext& context);
    static std::unique_ptr<OverwriteValueWithDifferentTypeTestBeh> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::OverwriteValueWithDifferentTypeTestBeh::create, OverwriteValueWithDifferentTypeTestBeh)
} /* namespace alica */
