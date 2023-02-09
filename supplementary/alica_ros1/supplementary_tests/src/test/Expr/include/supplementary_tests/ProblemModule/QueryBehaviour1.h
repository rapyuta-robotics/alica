#pragma once

#include <engine/constraintmodul/Query.h>
#include <mutex>
#include <supplementary_tests/DomainBehaviour.h>

namespace alica
{
class QueryBehaviour1 : public DomainBehaviour
{
public:
    QueryBehaviour1(BehaviourContext& context);
    virtual ~QueryBehaviour1();
    virtual void run();
    int getCallCounter();
    void stopQueries();

    static std::vector<double> result;
    std::shared_ptr<alica::Query> query;
    std::mutex queryMutex;
    bool stopQuerying;

protected:
    virtual void initialiseParameters();
    int callCounter;
};
} /* namespace alica */
