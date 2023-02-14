#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class SuccessOnInitBeh : public DomainBehaviour
{
public:
    SuccessOnInitBeh(BehaviourContext& context);
    virtual ~SuccessOnInitBeh();
    virtual void run();
    static std::unique_ptr<SuccessOnInitBeh> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::SuccessOnInitBeh::create, SuccessOnInitBeh)
} /* namespace alica */
