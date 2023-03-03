#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class DefendMid : public BasicBehaviour
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
