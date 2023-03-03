#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class SuccessOnInitBeh : public BasicBehaviour
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
