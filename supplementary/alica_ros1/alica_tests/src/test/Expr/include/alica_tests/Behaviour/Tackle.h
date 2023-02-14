#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class Tackle : public DomainBehaviour
{
public:
    Tackle(BehaviourContext& context);
    virtual ~Tackle();
    virtual void run();
    static std::unique_ptr<Tackle> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::Tackle::create, Tackle)
} /* namespace alica */
