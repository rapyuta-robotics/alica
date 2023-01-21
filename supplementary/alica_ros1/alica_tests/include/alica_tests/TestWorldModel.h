#pragma once

#include <alica/test/TestContext.h>

#include <atomic>
#include <string>
#include <unordered_map>
#include <vector>

namespace alicaTests
{

class TestWorldModelNew
{
public:
    TestWorldModelNew(alica::test::TestContext* tc)
            : _tc(tc)
    {
    }

    alica::test::TestContext* getTestContext() { return _tc; }

private:
    alica::test::TestContext* _tc;
};

class [[deprecated("Use TestWorldModelNew instead")]] TestWorldModel
{
public:
    TestWorldModel();
    virtual ~TestWorldModel();
    bool isTransitionCondition1413201227586() const;
    void setTransitionCondition1413201227586(bool transitionCondition1413201227586);
    bool isTransitionCondition1413201389955() const;
    void setTransitionCondition1413201389955(bool transitionCondition1413201389955);
    bool isTransitionCondition1413201052549() const;
    void setTransitionCondition1413201052549(bool transitionCondition1413201052549);
    bool isTransitionCondition1413201367990() const;
    void setTransitionCondition1413201367990(bool transitionCondition1413201367990);
    bool isTransitionCondition1413201370590() const;
    void setTransitionCondition1413201370590(bool transitionCondition1413201370590);

    bool isTransitionCondition1625614729978() const;
    void setTransitionCondition1625614729978(bool transitionCondition1625610805110);
    bool isTransitionCondition1625776897472() const;
    void setTransitionCondition1625776897472(bool transitionCondition1625776897472);
    bool isTransitionCondition1625783869825() const;
    void setTransitionCondition1625783869825(bool transitionCondition1625783869825);
    bool isTransitionCondition1625783867495() const;
    void setTransitionCondition1625783867495(bool transitionCondition1625783867495);
    bool isTransitionCondition1626848015861() const;
    void setTransitionCondition1626848015861(bool transitionCondition1626848015861);

    // MasterPlanTestSyncTransition
    bool isTransitionCondition1418825427317() const;
    void setTransitionCondition1418825427317(bool transitionCondition1418825427317);
    bool isTransitionCondition1418825428924() const;
    void setTransitionCondition1418825428924(bool transitionCondition1418825428924);

    bool isPreCondition1418042929966() const;
    void setPreCondition1418042929966(bool preCondition1418042929966);
    bool isRuntimeCondition1418042967134() const;
    void setRuntimeCondition1418042967134(bool runtimeCondition1418042967134);

    // Adjacent plans success test transitions
    bool isTransitionCondition1747408236004727286() const;
    void setTransitionCondition1747408236004727286(bool transitionCondition1747408236004727286);
    bool isTransitionCondition1067314038887345208() const;
    void setTransitionCondition1067314038887345208(bool transitionCondition1067314038887345208);

    // Test Tracing MasterPlan
    bool isPreCondition1840401110297459509() const;
    void setPreCondition1840401110297459509(bool preCondition1840401110297459509);

    bool isSwitchingEntryPoints() const;
    void setSwitchingEntryPoints(bool switchEntryPoints);

    std::vector<double> robotsXPos;
    double x;

    std::unordered_map<std::string, int> passedParameters;
    std::vector<std::string> configParameter;
    std::vector<std::pair<std::string, std::string>> tracingTags;
    std::vector<std::pair<std::string, std::string>> tracingLogs;
    std::unordered_map<std::string, std::string> tracingParents;

    void reset();

    // Failure handling tests
    void failurePlanInitCalled();
    int failurePlanInitCallCount() const;
    void enableTransitionCondition3194919312481305139();
    bool transitionCondition3194919312481305139Enabled() const;
    void setTransitionCondition1446293122737278544(bool value);
    bool isTransitionCondition1446293122737278544() const;
    void setTransitionCondition1023566846009251524(bool value);
    bool isTransitionCondition1023566846009251524() const;

private:
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

    // tracing master plan
    bool preCondition1840401110297459509;

    // Adjacent plans success test
    bool transitionCondition1747408236004727286;
    bool transitionCondition1067314038887345208;

    bool switchEntryPoints;

    // Failure handling tests
    std::atomic<int> failurePlanInitCallCounter;
    std::atomic<bool> transitionCondition3194919312481305139;
    std::atomic<bool> transitionCondition1446293122737278544;
    std::atomic<bool> transitionCondition1023566846009251524;
};

} // namespace alicaTests
