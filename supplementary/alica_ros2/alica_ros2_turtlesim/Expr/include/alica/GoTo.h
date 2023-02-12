#pragma once

#include <alica/DomainBehaviour.h>
#include <engine/constraintmodul/Query.h>

namespace alica
{
class GoTo : public DomainBehaviour
{
public:
    GoTo(BehaviourContext& context);
    virtual ~GoTo();
    virtual void run();
protected:
    virtual void initialiseParameters();
private:
    alica::Query _query;
    std::vector<double> _results;
};
} /* namespace alica */
