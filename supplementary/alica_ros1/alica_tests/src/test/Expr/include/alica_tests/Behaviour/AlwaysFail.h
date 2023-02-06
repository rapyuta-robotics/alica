#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class AlwaysFail : public DomainBehaviour
{
public:
    AlwaysFail(BehaviourContext& context);
    virtual ~AlwaysFail();
    virtual void run();
    static std::unique_ptr<AlwaysFail> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};

BOOST_DLL_ALIAS(alica::AlwaysFail::create, AlwaysFail)

} /* namespace alica */
