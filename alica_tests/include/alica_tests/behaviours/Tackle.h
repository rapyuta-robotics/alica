#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class Tackle : public BasicBehaviour
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
