#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
// used for accessing bb values with an unknown type
class UnknownType
{
public:
    UnknownType() = default;
    UnknownType(int64_t input)
            : value(input){};
    int64_t value;
};

class BlackboardExceptionTestBeh : public BasicBehaviour
{
public:
    BlackboardExceptionTestBeh(BehaviourContext& context);
    static std::unique_ptr<BlackboardExceptionTestBeh> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::BlackboardExceptionTestBeh::create, BlackboardExceptionTestBeh)
} /* namespace alica */
