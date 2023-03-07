#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class ValueMappingTestBeh : public BasicBehaviour
{
public:
    ValueMappingTestBeh(BehaviourContext& context);
    virtual void run();
    static std::unique_ptr<ValueMappingTestBeh> create(alica::BehaviourContext& context);

private:
};
BOOST_DLL_ALIAS(alica::ValueMappingTestBeh::create, ValueMappingTestBeh)
} /* namespace alica */
