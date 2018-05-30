/*
 * TestWorldModel.cpp
 *
 *  Created on: Oct 14, 2014
 *      Author: Stefan Jakob
 */

#include "TestWorldModel.h"

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
    : trigger1(nullptr)
    , trigger2(nullptr)
{
    this->transitionCondition1413201227586 = false;
    this->transitionCondition1413201389955 = false;
    this->transitionCondition1413201052549 = false;
    this->transitionCondition1413201367990 = false;
    this->transitionCondition1413201370590 = false;
    this->preCondition1418042929966 = false;
    this->runtimeCondition1418042967134 = false;

    this->transitionCondition1418825427317 = false;
    this->transitionCondition1418825428924 = false;

    this->x = 0;
}

TestWorldModel::~TestWorldModel()
{
    // TODO Auto-generated destructor stub
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

} // namespace alicaTests
