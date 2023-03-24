#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>
#include <engine/constraintmodul/Query.h>
#include <vector>

namespace alica
{
class ConstraintUsingBehaviour : public BasicBehaviour
{
public:
    ConstraintUsingBehaviour(BehaviourContext& context);
    virtual ~ConstraintUsingBehaviour();
    virtual void run();
    static std::unique_ptr<ConstraintUsingBehaviour> create(alica::BehaviourContext& context);
    int getCallCounter() const;
    static std::vector<int64_t> result;

protected:
    virtual void initialiseParameters();
    Query _query;
    int _callCounter;
};
BOOST_DLL_ALIAS(alica::ConstraintUsingBehaviour::create, ConstraintUsingBehaviour)
} /* namespace alica */
