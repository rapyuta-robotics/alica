#include "TestWorldModel.h"

#include <essentials/EventTrigger.h>

namespace alicaTests
{

TestWorldModel* TestWorldModel::getOne()
{
    static TestWorldModel instance;
    return &instance;
}

TestWorldModel* TestWorldModel::getTwo()
{
    static TestWorldModel instance;
    return &instance;
}

TestWorldModel::TestWorldModel()
        : trigger1(new essentials::EventTrigger())
        , trigger2(new essentials::EventTrigger())
{
    reset();
}

TestWorldModel::~TestWorldModel()
{
}

void TestWorldModel::reset()
{
    transitionCondition1413201227586 = false;
    transitionCondition1413201389955 = false;
    transitionCondition1413201052549 = false;
    transitionCondition1413201367990 = false;
    transitionCondition1413201370590 = false;
    preCondition1418042929966 = false;
    runtimeCondition1418042967134 = false;

    transitionCondition1418825427317 = false;
    transitionCondition1418825428924 = false;

    transitionCondition1625610805110 = false;

    x = 0;
}

bool TestWorldModel::isTransitionCondition1413201227586()
{
    return transitionCondition1413201227586;
}

void TestWorldModel::setTransitionCondition1413201227586(bool transitionCondition1413201227586)
{
    this->transitionCondition1413201227586 = transitionCondition1413201227586;
}

bool TestWorldModel::isTransitionCondition1413201389955()
{
    return transitionCondition1413201389955;
}

void TestWorldModel::setTransitionCondition1413201389955(bool transitionCondition1413201389955)
{
    this->transitionCondition1413201389955 = transitionCondition1413201389955;
}

bool TestWorldModel::isTransitionCondition1413201052549()
{
    return transitionCondition1413201052549;
}

void TestWorldModel::setTransitionCondition1413201052549(bool transitionCondition1413201052549)
{
    this->transitionCondition1413201052549 = transitionCondition1413201052549;
}

bool TestWorldModel::isTransitionCondition1413201367990()
{
    return transitionCondition1413201367990;
}

void TestWorldModel::setTransitionCondition1413201367990(bool transitionCondition1413201367990)
{
    this->transitionCondition1413201367990 = transitionCondition1413201367990;
}

bool TestWorldModel::isTransitionCondition1413201370590()
{
    return transitionCondition1413201370590;
}

void TestWorldModel::setTransitionCondition1413201370590(bool transitionCondition1413201370590)
{
    this->transitionCondition1413201370590 = transitionCondition1413201370590;
}

bool TestWorldModel::isTransitionCondition1418825427317()
{
    return this->transitionCondition1418825427317;
}
void TestWorldModel::setTransitionCondition1418825427317(bool transitionCondition1418825427317)
{
    this->transitionCondition1418825427317 = transitionCondition1418825427317;
}
bool TestWorldModel::isTransitionCondition1418825428924()
{
    return this->transitionCondition1418825428924;
}
void TestWorldModel::setTransitionCondition1418825428924(bool transitionCondition1418825428924)
{
    this->transitionCondition1418825428924 = transitionCondition1418825428924;
}

bool TestWorldModel::isPreCondition1418042929966()
{
    return preCondition1418042929966;
}
void TestWorldModel::setPreCondition1418042929966(bool preCondition1418042929966)
{
    this->preCondition1418042929966 = preCondition1418042929966;
}
bool TestWorldModel::isRuntimeCondition1418042967134()
{
    return runtimeCondition1418042967134;
}
void TestWorldModel::setRuntimeCondition1418042967134(bool runtimeCondition1418042967134)
{
    this->runtimeCondition1418042967134 = runtimeCondition1418042967134;
}

bool TestWorldModel::isTransitionCondition1625610805110()
{
    return this->transitionCondition1625610805110;
}

void TestWorldModel::setTransitionCondition1625610805110(bool transitionCondition1625610805110)
{
    this->transitionCondition1625610805110 = transitionCondition1625610805110;
}

} // namespace alicaTests
