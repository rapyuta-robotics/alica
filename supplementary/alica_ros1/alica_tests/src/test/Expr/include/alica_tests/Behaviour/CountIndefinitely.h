#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class CountIndefinitely : public DomainBehaviour
{
public:
    CountIndefinitely(BehaviourContext& context);
    virtual ~CountIndefinitely();
    virtual void run();
    static std::unique_ptr<CountIndefinitely> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::CountIndefinitely::create, CountIndefinitely)
} /* namespace alica */
