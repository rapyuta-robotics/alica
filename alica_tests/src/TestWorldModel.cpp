#include <alica_tests/TestWorldModel.h>

namespace alicaTests
{

std::atomic<uint64_t> TestWorldModel::_curAgentNo;

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
{
    reset();
}

TestWorldModel::~TestWorldModel() {}

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

    transitionCondition1625614729978 = false;
    transitionCondition1625776897472 = false;
    switchEntryPoints = false;

    transitionCondition4238964946542987247 = false;
    transitionCondition4115970455290610262 = false;
    preCondition1840401110297459509 = false;

    transitionCondition4496654201854254411 = false;
    transitionCondition3126176581533900616 = false;
    transitionCondition1078898265232036813 = false;
    transitionCondition4344644064496100420 = false;

    _curAgentNo = 111111111111111;

    x = 0;
    tracingLogs.clear();
    tracingTags.clear();
    tracingParents.clear();

    serializationTestA = false;
    serializationTestB = false;
    serializationTestC = false;
    serializationTestD = false;
}

bool TestWorldModel::isTransitionCondition1413201227586() const
{
    return transitionCondition1413201227586;
}

void TestWorldModel::setTransitionCondition1413201227586(bool transitionCondition1413201227586)
{
    this->transitionCondition1413201227586 = transitionCondition1413201227586;
}

bool TestWorldModel::isTransitionCondition1413201389955() const
{
    return transitionCondition1413201389955;
}

void TestWorldModel::setTransitionCondition1413201389955(bool transitionCondition1413201389955)
{
    this->transitionCondition1413201389955 = transitionCondition1413201389955;
}

bool TestWorldModel::isTransitionCondition1413201052549() const
{
    return transitionCondition1413201052549;
}

void TestWorldModel::setTransitionCondition1413201052549(bool transitionCondition1413201052549)
{
    this->transitionCondition1413201052549 = transitionCondition1413201052549;
}

bool TestWorldModel::isTransitionCondition1413201367990() const
{
    return transitionCondition1413201367990;
}

void TestWorldModel::setTransitionCondition1413201367990(bool transitionCondition1413201367990)
{
    this->transitionCondition1413201367990 = transitionCondition1413201367990;
}

bool TestWorldModel::isTransitionCondition1413201370590() const
{
    return transitionCondition1413201370590;
}

void TestWorldModel::setTransitionCondition1413201370590(bool transitionCondition1413201370590)
{
    this->transitionCondition1413201370590 = transitionCondition1413201370590;
}

bool TestWorldModel::isTransitionCondition1418825427317() const
{
    return this->transitionCondition1418825427317;
}
void TestWorldModel::setTransitionCondition1418825427317(bool transitionCondition1418825427317)
{
    this->transitionCondition1418825427317 = transitionCondition1418825427317;
}
bool TestWorldModel::isTransitionCondition1418825428924() const
{
    return this->transitionCondition1418825428924;
}
void TestWorldModel::setTransitionCondition1418825428924(bool transitionCondition1418825428924)
{
    this->transitionCondition1418825428924 = transitionCondition1418825428924;
}

bool TestWorldModel::isPreCondition1418042929966() const
{
    return preCondition1418042929966;
}
void TestWorldModel::setPreCondition1418042929966(bool preCondition1418042929966)
{
    this->preCondition1418042929966 = preCondition1418042929966;
}
bool TestWorldModel::isRuntimeCondition1418042967134() const
{
    return runtimeCondition1418042967134;
}
void TestWorldModel::setRuntimeCondition1418042967134(bool runtimeCondition1418042967134)
{
    this->runtimeCondition1418042967134 = runtimeCondition1418042967134;
}

bool TestWorldModel::isTransitionCondition1625614729978() const
{
    return this->transitionCondition1625614729978;
}

void TestWorldModel::setTransitionCondition1625614729978(bool transitionCondition1625614729978)
{
    this->transitionCondition1625614729978 = transitionCondition1625614729978;
}

bool TestWorldModel::isTransitionCondition1625776897472() const
{
    return this->transitionCondition1625776897472;
}

void TestWorldModel::setTransitionCondition1625776897472(bool transitionCondition1625776897472)
{
    this->transitionCondition1625776897472 = transitionCondition1625776897472;
}

bool TestWorldModel::isTransitionCondition1625783869825() const
{
    return this->transitionCondition1625783869825;
}

void TestWorldModel::setTransitionCondition1625783869825(bool transitionCondition1625783869825)
{
    this->transitionCondition1625783869825 = transitionCondition1625783869825;
}

bool TestWorldModel::isTransitionCondition1625783867495() const
{
    return this->transitionCondition1625783867495;
}

void TestWorldModel::setTransitionCondition1625783867495(bool transitionCondition1625783867495)
{
    this->transitionCondition1625783867495 = transitionCondition1625783867495;
}

bool TestWorldModel::isTransitionCondition1626848015861() const
{
    return this->transitionCondition1626848015861;
}

void TestWorldModel::setTransitionCondition1626848015861(bool transitionCondition1626848015861)
{
    this->transitionCondition1626848015861 = transitionCondition1626848015861;
}

bool TestWorldModel::isTransitionCondition4238964946542987247() const
{
    return this->transitionCondition4238964946542987247;
}

void TestWorldModel::setTransitionCondition4238964946542987247(bool transitionCondition4238964946542987247)
{
    this->transitionCondition4238964946542987247 = transitionCondition4238964946542987247;
}

bool TestWorldModel::isTransitionCondition4115970455290610262() const
{
    return this->transitionCondition4115970455290610262;
}

void TestWorldModel::setTransitionCondition4115970455290610262(bool transitionCondition4115970455290610262)
{
    this->transitionCondition4115970455290610262 = transitionCondition4115970455290610262;
}

bool TestWorldModel::isSwitchingEntryPoints() const
{
    return this->switchEntryPoints;
}

void TestWorldModel::setSwitchingEntryPoints(bool switchEntryPoints)
{
    this->switchEntryPoints = switchEntryPoints;
}

bool TestWorldModel::isTransitionCondition4496654201854254411() const
{
    return this->transitionCondition4496654201854254411;
}

void TestWorldModel::setTransitionCondition4496654201854254411(bool new_val)
{
    this->transitionCondition4496654201854254411 = new_val;
}

bool TestWorldModel::isTransitionCondition1078898265232036813() const
{
    return transitionCondition1078898265232036813;
}

void TestWorldModel::setTransitionCondition1078898265232036813(bool val)
{
    transitionCondition1078898265232036813 = val;
}

bool TestWorldModel::isTransitionCondition3126176581533900616() const
{
    return transitionCondition3126176581533900616;
}

void TestWorldModel::setTransitionCondition3126176581533900616(bool val)
{
    transitionCondition3126176581533900616 = val;
}

bool TestWorldModel::isTransitionCondition1747408236004727286() const
{
    return this->transitionCondition1747408236004727286;
}
void TestWorldModel::setTransitionCondition1747408236004727286(bool transitionCondition1747408236004727286)
{
    this->transitionCondition1747408236004727286 = transitionCondition1747408236004727286;
}

bool TestWorldModel::isTransitionCondition1067314038887345208() const
{
    return this->transitionCondition1067314038887345208;
}
void TestWorldModel::setTransitionCondition1067314038887345208(bool transitionCondition1067314038887345208)
{
    this->transitionCondition1067314038887345208 = transitionCondition1067314038887345208;
}

bool TestWorldModel::isPreCondition1840401110297459509() const
{
    return this->preCondition1840401110297459509;
}

void TestWorldModel::setPreCondition1840401110297459509(bool preCondition1840401110297459509)
{
    this->preCondition1840401110297459509 = preCondition1840401110297459509;
}

bool TestWorldModel::isTransitionCondition4344644064496100420() const
{
    return this->transitionCondition4344644064496100420;
}

void TestWorldModel::setTransitionCondition4344644064496100420(bool transitionCondition4344644064496100420)
{
    this->transitionCondition4344644064496100420 = transitionCondition4344644064496100420;
}

} // namespace alicaTests
