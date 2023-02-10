#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class ReadConfigurationBehaviour : public DomainBehaviour
{
public:
    ReadConfigurationBehaviour(BehaviourContext& context);
    virtual ~ReadConfigurationBehaviour();
    virtual void run();
    static std::unique_ptr<ReadConfigurationBehaviour> create(alica::BehaviourContext& context);

    std::string testValue;

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::ReadConfigurationBehaviour::create, ReadConfigurationBehaviour)
} /* namespace alica */
