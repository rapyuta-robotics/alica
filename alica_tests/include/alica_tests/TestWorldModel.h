#pragma once

#include <engine/IAlicaWorldModel.h>

#include <vector>
#include <string>
#include <unordered_map>

namespace alicaTests
{

class TestWorldModel : public alica::IAlicaWorldModel
{
public:
    TestWorldModel();
    virtual ~TestWorldModel();
    static TestWorldModel* getOne();
    static TestWorldModel* getTwo();
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

    // PlanPool test transitions
    bool isTransitionCondition4238964946542987247() const;
    void setTransitionCondition4238964946542987247(bool transitionCondition4238964946542987247);
    bool isTransitionCondition4115970455290610262() const;
    void setTransitionCondition4115970455290610262(bool transitionCondition4115970455290610262);

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
    bool isTransitionCondition1914245867924544479() const;
    void setTransitionCondition1914245867924544479(bool transitionCondition1914245867924544479);
    bool isTransitionCondition3345031375302716643() const;
    void setTransitionCondition3345031375302716643(bool transitionCondition3345031375302716643);
    bool isTransitionCondition1390829819585906015() const;
    void setTransitionCondition1390829819585906015(bool transitionCondition1390829819585906015);
    bool isTransitionCondition3143778092687974738() const;
    void setTransitionCondition3143778092687974738(bool transitionCondition3143778092687974738);
    
    // Test Tracing MasterPlan
    bool isPreCondition1840401110297459509() const;
    void setPreCondition1840401110297459509(bool preCondition1840401110297459509);

    bool isSwitchingEntryPoints() const;
    void setSwitchingEntryPoints(bool switchEntryPoints);
    bool doWait();
    void setWait(bool wait);

    std::vector<double> robotsXPos;
    double x;

    std::vector<std::string> configParameter;
    std::vector<std::pair<std::string, std::string>> tracingTags;
    std::vector<std::pair<std::string, std::string>> tracingLogs;
    std::unordered_map<std::string, std::string> tracingParents;

    void reset();

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

    // PlanPoolTest transitions
    bool transitionCondition4238964946542987247;
    bool transitionCondition4115970455290610262;
    //tracing master plan
    bool preCondition1840401110297459509;

    // Adjacent plans success test
    bool transitionCondition1914245867924544479;
    bool transitionCondition3345031375302716643;
    bool transitionCondition1390829819585906015;
    bool transitionCondition3143778092687974738;

    bool switchEntryPoints;
    bool wait;
};

} // namespace alicaTests
