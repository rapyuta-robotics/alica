#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class SuccessSpam : public BasicBehaviour
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
