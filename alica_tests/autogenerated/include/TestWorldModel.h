#pragma once

#include <supplementary/ITrigger.h>
#include <SystemConfig.h>

#include <vector>

namespace alicaTests {

class TestWorldModel {
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

    // MasterPlanTestSyncTransition
    bool isTransitionCondition1418825427317();
    void setTransitionCondition1418825427317(bool transitionCondition1418825427317);
    bool isTransitionCondition1418825428924();
    void setTransitionCondition1418825428924(bool transitionCondition1418825428924);

    bool isPreCondition1418042929966();
    void setPreCondition1418042929966(bool preCondition1418042929966);
    bool isRuntimeCondition1418042967134();
    void setRuntimeCondition1418042967134(bool runtimeCondition1418042967134);

    std::vector<double> robotsXPos;
    double x;
    supplementary::ITrigger* trigger1;
    supplementary::ITrigger* trigger2;

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
};

}  // namespace alicaTests
