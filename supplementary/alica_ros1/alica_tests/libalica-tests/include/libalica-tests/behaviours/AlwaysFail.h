#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class AlwaysFail : public BasicBehaviour
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
