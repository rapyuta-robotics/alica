#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class DefendMid : public DomainBehaviour
{
public:
    DefendMid(BehaviourContext& context);
    virtual ~DefendMid();
    virtual void run();
    static std::unique_ptr<DefendMid> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::DefendMid::create, DefendMid)
} /* namespace alica */
