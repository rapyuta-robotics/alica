#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class MidFieldStandard : public DomainBehaviour
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
