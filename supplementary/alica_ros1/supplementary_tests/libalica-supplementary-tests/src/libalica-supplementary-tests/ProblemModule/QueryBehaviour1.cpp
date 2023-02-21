#include <libalica-supplementary-tests/ProblemModule/QueryBehaviour1.h>
#include <memory>

#include <constraintsolver/CGSolver.h>

namespace alica
{
std::vector<double> QueryBehaviour1::result;

QueryBehaviour1::QueryBehaviour1(BehaviourContext& context)
        : BasicBehaviour(context)
{

    this->callCounter = 0;
}

void QueryBehaviour1::run()
{

    callCounter++;

    std::lock_guard<std::mutex> guard(queryMutex);
    if (!stopQuerying) {
        query->getSolution<reasoner::CGSolver, double>(getPlanContext(), result);
    }
}

void QueryBehaviour1::initialiseParameters()
{

    this->query = std::make_shared<alica::Query>();
    query->clearStaticVariables();
    query->addStaticVariable(getVariable("QBX"));
    query->addStaticVariable(getVariable("QBY"));
    stopQuerying = false;
}

void QueryBehaviour1::stopQueries()
{
    std::lock_guard<std::mutex> guard(queryMutex);
    stopQuerying = true;
}

int QueryBehaviour1::getCallCounter()
{
    return callCounter;
}

std::unique_ptr<QueryBehaviour1> QueryBehaviour1::create(alica::BehaviourContext& context)
{
    return std::make_unique<QueryBehaviour1>(context);
}
} /* namespace alica */
