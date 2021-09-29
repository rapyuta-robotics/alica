#pragma once

#include <essentials/ITrigger.hpp>

#include <vector>

namespace alicaTests
{

class TestWorldModel
{
public:
    virtual ~TestWorldModel();
    static TestWorldModel* getOne();
    static TestWorldModel* getTwo();
    bool isTransitionCondition1413201227586();
    void setTransitionCondition1413201227586(bool transitionCondition1413201227586);
    bool isTransitionCondition1413201389955();
    void setTransitionCondition1413201389955(bool transitionCondition1413201389955);
    bool isTransitionCondition1413201052549();
    void setTransitionCondition1413201052549(bool transitionCondition1413201052549);
    bool isTransitionCondition1413201367990();
    void setTransitionCondition1413201367990(bool transitionCondition1413201367990);
    bool isTransitionCondition1413201370590();
    void setTransitionCondition1413201370590(bool transitionCondition1413201370590);

    bool isTransitionCondition1625614729978();
    void setTransitionCondition1625614729978(bool transitionCondition1625610805110);
    bool isTransitionCondition1625776897472();
    void setTransitionCondition1625776897472(bool transitionCondition1625776897472);
    bool isTransitionCondition1625783869825();
    void setTransitionCondition1625783869825(bool transitionCondition1625783869825);
    bool isTransitionCondition1625783867495();
    void setTransitionCondition1625783867495(bool transitionCondition1625783867495);
    bool isTransitionCondition1626848015861();
    void setTransitionCondition1626848015861(bool transitionCondition1626848015861);

    // MasterPlanTestSyncTransition
    bool isTransitionCondition1418825427317();
    void setTransitionCondition1418825427317(bool transitionCondition1418825427317);
    bool isTransitionCondition1418825428924();
    void setTransitionCondition1418825428924(bool transitionCondition1418825428924);

    bool isPreCondition1418042929966();
    void setPreCondition1418042929966(bool preCondition1418042929966);
    bool isRuntimeCondition1418042967134();
    void setRuntimeCondition1418042967134(bool runtimeCondition1418042967134);

    bool isSwitchingEntryPoints();
    void setSwitchingEntryPoints(bool switchEntryPoints);

    std::vector<double> robotsXPos;
    double x;
    std::shared_ptr<essentials::ITrigger> trigger1;
    std::shared_ptr<essentials::ITrigger> trigger2;

    void reset();

private:
    TestWorldModel();
    bool transitionCondition1413201227586;
    bool transitionCondition1413201389955;
    bool transitionCondition1413201052549;
    bool transitionCondition1413201367990;
    bool transitionCondition1413201370590;

    // SyncTransitionTest
    bool transitionCondition1418825427317;
    bool transitionCondition1418825428924;

    // PlanTypeTest
    bool preCondition1418042929966;
    bool runtimeCondition1418042967134;

    // Engine rules scheduling test
    bool transitionCondition1625614729978;
    // Engine rules scheduling test failure transition
    bool transitionCondition1625776897472;
    // Top level failure
    bool transitionCondition1625783869825;
    // SubPlan
    bool transitionCondition1625783867495;
    // master plan final transition
    bool transitionCondition1626848015861;

    bool switchEntryPoints;
};

} // namespace alicaTests