#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class SuccessSpam : public DomainBehaviour
{
public:
    SuccessSpam(BehaviourContext& context);
    virtual ~SuccessSpam();
    virtual void run();
    static std::unique_ptr<SuccessSpam> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::SuccessSpam::create, SuccessSpam)
} /* namespace alica */
