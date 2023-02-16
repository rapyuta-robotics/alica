#pragma once

#include <boost/dll/alias.hpp>
#include <engine/constraintmodul/Query.h>
#include <mutex>
#include <engine/BasicBehaviour.h>

namespace alica
{
class QueryBehaviour1 : public BasicBehaviour
{
public:
    QueryBehaviour1(BehaviourContext& context);
    virtual ~QueryBehaviour1();
    virtual void run();
    static std::unique_ptr<QueryBehaviour1> create(alica::BehaviourContext& context);
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
BOOST_DLL_ALIAS(alica::QueryBehaviour1::create, QueryBehaviour1)
} /* namespace alica */
