#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class MidFieldStandard : public BasicBehaviour
{
public:
    MidFieldStandard(BehaviourContext& context);
    virtual ~MidFieldStandard();
    virtual void run();
    static std::unique_ptr<MidFieldStandard> create(alica::BehaviourContext& context);

    int callCounter;

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::MidFieldStandard::create, MidFieldStandard)
} /* namespace alica */
