using namespace std;

#include "SystemConfig.h"
#include <cstdio>
#include <gtest/gtest.h>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <test_alica.h>
#include <typeinfo>

#include "engine/AlicaClock.h"
#include "engine/BasicBehaviour.h"
#include "engine/BehaviourPool.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/PlanRepository.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/AlicaElement.h"
#include "engine/model/Behaviour.h"
#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/Condition.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/ForallAgents.h"
#include "engine/model/Plan.h"
#include "engine/model/PostCondition.h"
#include "engine/model/PreCondition.h"
#include "engine/model/Quantifier.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/State.h"
#include "engine/model/SyncTransition.h"
#include "engine/model/Task.h"
#include "engine/model/TerminalState.h"
#include "engine/model/Transition.h"
#include "engine/parser/PlanWriter.h"
#include <communication/AlicaRosCommunication.h>
#include <engine/AlicaEngine.h>

class AlicaEngineTest : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "MasterPlan"; }
    bool stepEngine() const override { return false; }

    static std::string exec(const char* cmd)
    {
        FILE* pipe = popen(cmd, "r");
        if (!pipe)
            return "ERROR";
        char buffer[128];
        std::string result = "";
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
        pclose(pipe);
        return result;
    }

    static void checkAlicaElement(const alica::AlicaElement* ae, long id, string name, string comment)
    {
        EXPECT_EQ(id, ae->getId()) << "Wrong ID!" << endl;
        EXPECT_STREQ(name.c_str(), ae->getName().c_str()) << "Wrong Name for Element of type " << endl;
    }

    static void checkState(const alica::State* s, long id, string name, string comment, initializer_list<long> absPlanIDs, initializer_list<long> inTransitions,
            initializer_list<long> outTransitions, long entryPointID = 0)
    {
        checkAlicaElement(s, id, name, comment);
        if (entryPointID != 0) {
            EXPECT_EQ(entryPointID, s->getEntryPoint()->getId()) << "Wrong EntryPoint for state!" << endl;
        }
        EXPECT_EQ(absPlanIDs.size(), s->getPlans().size()) << "Number of abstractPlans didnt fit plans size." << endl;
        for (const alica::AbstractPlan* p : s->getPlans()) {
            EXPECT_TRUE(find(absPlanIDs.begin(), absPlanIDs.end(), p->getId()) != absPlanIDs.end()) << "Unknown id for AbstractPlan!" << endl;
        }
        if (inTransitions.size() != 0) {
            for (const alica::Transition* t : s->getInTransitions()) {
                EXPECT_TRUE(find(inTransitions.begin(), inTransitions.end(), t->getId()) != inTransitions.end()) << "Unknown id for InTransition!" << endl;
            }
        } else {
            EXPECT_NE(nullptr, s->getEntryPoint()) << "Isolated state found!";
        }
        if (outTransitions.size() != 0) {
            for (const alica::Transition* t : s->getOutTransitions()) {
                EXPECT_TRUE(find(outTransitions.begin(), outTransitions.end(), t->getId()) != outTransitions.end()) << "Unknown id for OutTransition!" << endl;
            }
        }
    }
    static void checkEntryPoint(const alica::EntryPoint* ep, long id, string name, string comment, bool successRequired, int minCardinality, int maxCardinality,
            long stateID, long taskID, string taskName)
    {
        checkAlicaElement(ep, id, name, comment);

        EXPECT_EQ(successRequired, ep->isSuccessRequired()) << "SuccesRequired true instead of false!" << endl;
        EXPECT_EQ(minCardinality, ep->getMinCardinality()) << "Wrong minCardinality ID!" << endl;
        EXPECT_EQ(maxCardinality, ep->getMaxCardinality()) << "Wrong maxCardinality ID!" << endl;
        EXPECT_EQ(stateID, ep->getState()->getId()) << "Wrong stateId for EntryPoint!" << endl;
        EXPECT_EQ(taskID, ep->getTask()->getId()) << "Wrong TaskId for EntryPoint!" << endl;
        EXPECT_STREQ(taskName.c_str(), ep->getTask()->getName().c_str()) << "Wrong taskName!" << endl;
    }

    static void checkPlan(
            const alica::Plan* plan, long id, string name, string comment, bool masterPlan, double utilityThreshold, int minCardinality, int maxCardinality)
    {
        checkAlicaElement(plan, id, name, comment);
        EXPECT_EQ(masterPlan, plan->isMasterPlan()) << "MasterPlan true instead of false!" << endl;
        EXPECT_EQ(utilityThreshold, plan->getUtilityThreshold()) << "Wrong utilityThreshold!" << endl;
        EXPECT_EQ(minCardinality, plan->getMinCardinality()) << "Wrong minCardinality!" << endl;
        EXPECT_EQ(maxCardinality, plan->getMaxCardinality()) << "Wrong maxCardinality!" << endl;
    }

    static void checkPreCondition(const alica::PreCondition* condition, long id, string name, string comment, string conString, string pluginName, bool enabled)
    {
        checkAlicaElement(condition, id, name, comment);
        EXPECT_STREQ(conString.c_str(), condition->getConditionString().c_str()) << "Wrong ConditionString!" << endl;
        EXPECT_STREQ(condition->getPlugInName().c_str(), pluginName.c_str()) << "Wrong PlugInName!" << endl;
        EXPECT_EQ(enabled, condition->isEnabled()) << "Wrong enabled value!" << endl;
    }

    static void checkPostCondition(const alica::PostCondition* condition, long id, string name, string comment, string conString, string pluginName)
    {
        checkAlicaElement(condition, id, name, comment);
        EXPECT_STREQ(conString.c_str(), condition->getConditionString().c_str()) << "Wrong ConditionString!" << endl;
        EXPECT_STREQ(condition->getPlugInName().c_str(), pluginName.c_str()) << "Wrong PlugInName!" << endl;
    }

    static void checkRuntimeCondition(const alica::RuntimeCondition* condition, long id, string name, string comment, string conString, string pluginName)
    {
        checkAlicaElement(condition, id, name, comment);
        EXPECT_STREQ(conString.c_str(), condition->getConditionString().c_str()) << "Wrong ConditionString!" << endl;
        EXPECT_STREQ(condition->getPlugInName().c_str(), pluginName.c_str()) << "Wrong PlugInName!" << endl;
    }
    static void checkTransition(const alica::Transition* transition, long id, string name, string comment, long preConditionId, long inState, long outState,
            string preConName, string preConComment, string preConString, string pluginName, bool enabled)
    {
        checkAlicaElement(transition, id, name, comment);
        checkPreCondition(transition->getPreCondition(), preConditionId, preConName, preConComment, preConString, pluginName, enabled);

        EXPECT_EQ(inState, transition->getInState()->getId()) << "Unknown id for InState!" << endl;

        EXPECT_EQ(outState, transition->getOutState()->getId()) << "Unknown id for OutState!" << endl;
    }

    static void checkSyncTransition(const alica::SyncTransition* transition, long id, string name, string comment, int talkTimeout, int syncTimeout)
    {
        checkAlicaElement(transition, id, name, comment);
        EXPECT_EQ(AlicaTime::milliseconds(talkTimeout), transition->getTalkTimeOut()) << "Wrong talkTimeout!" << endl;
        EXPECT_EQ(AlicaTime::milliseconds(syncTimeout), transition->getSyncTimeOut()) << "Wrong syncTimeout!" << endl;
    }

    static void checkQuantifier(const alica::Quantifier* quantifier, long id, string name, string comment, long scope, initializer_list<string> sorts)
    {
        checkAlicaElement(quantifier, id, name, comment);
        EXPECT_EQ(scope, quantifier->getScope()->getId()) << "Wrong Scope" << endl;
        for (string sorts : quantifier->getDomainIdentifiers()) {
            EXPECT_TRUE(find(quantifier->getDomainIdentifiers().begin(), quantifier->getDomainIdentifiers().end(), sorts.c_str()) !=
                        quantifier->getDomainIdentifiers().end())
                    << "Sort not found!" << endl;
        }
    }
};

/**
 * Tests the plan parser with some nice plans
 */
TEST_F(AlicaEngineTest, planParser)
{
    ASSERT_NO_SIGNAL

    const auto& plans = ae->getPlanRepository()->getPlans();

    cout << "Printing plans from Repository: " << endl;
    for (const Plan* plan : plans) {
        cout << "--------- Next Plan: -------------" << endl;
        cout << "ID: " << plan->getId() << endl;
        cout << "Plan: " << plan->toString() << endl;
        EXPECT_TRUE(plan->getId() == 1402488634525 || plan->getId() == 1402488893641 || plan->getId() == 1402488870347 || plan->getId() == 1402488437260 ||
                    plan->getId() == 1402488770050 || plan->getId() == 1402489318663)
                << "ID not part of testplans!" << endl;
        switch (plan->getId()) {
        case 1402488634525:
            checkPlan(plan, 1402488634525, "AttackPlan", "", false, 0.1, 0, 2147483647);
            cout << "States: " << endl;
            EXPECT_EQ(2, plan->getStates().size()) << "Number of states didnt fit AttackPlan.pml state size." << endl;
            for (const alica::State* s : plan->getStates()) {
                cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                switch (s->getId()) {
                case 1402488646220:
                    checkState(s, 1402488646220, "Attack", "", {1402489366699, 1402489318663}, {1402489460694}, {1402489459382}, 1402488646221);
                    break;
                case 1402489396914:
                    checkState(s, 1402489396914, "Shoot", "", {1402488866727}, {1402489459382}, {1402489460694});
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a state not part of AttackPlan.pml!" << endl;
                    break;
                }
            }
            cout << "Transitions: " << endl;
            EXPECT_EQ(2, plan->getTransitions().size()) << "Number of Transitions didnt fit AttackPlan.pml EntryPoints size." << endl;
            for (const alica::Transition* t : plan->getTransitions()) {
                cout << "\t" << t->getName() << " ID: " << t->getId() << endl;
                switch (t->getId()) {
                case 1402489459382:
                    checkTransition(
                            t, 1402489459382, "MISSING_NAME", "", 1402489460549, 1402488646220, 1402489396914, "MISSING_NAME", "", "", "DefaultPlugin", true);
                    cout << "Quantifiers: " << endl;
                    for (const alica::Quantifier* q : t->getPreCondition()->getQuantifiers()) {
                        switch (q->getId()) {
                        case 1403773214317:
                            cout << "\t" << q->getName() << " ID: " << q->getId() << endl;
                            checkQuantifier(q, 1403773214317, "MISSING_NAME", "", 1402488634525, {"X", "Y"});
                            EXPECT_TRUE(dynamic_cast<const alica::ForallAgents*>(q) != 0) << "Wrong Type!" << endl;
                            break;
                        case 1403773224776:
                            cout << "\t" << q->getName() << " ID: " << q->getId() << endl;
                            checkQuantifier(q, 1403773224776, "MISSING_NAME", "", 1402488646220, {"A", "B"});
                            EXPECT_TRUE(dynamic_cast<const alica::ForallAgents*>(q) != 0) << "Wrong Type!" << endl;
                            break;
                        case 1403773234841:
                            cout << "\t" << q->getName() << " ID: " << q->getId() << endl;
                            checkQuantifier(q, 1403773234841, "MISSING_NAME", "", 1402489396914, {"another one"});
                            EXPECT_TRUE(dynamic_cast<const alica::ForallAgents*>(q) != 0) << "Wrong Type!" << endl;
                            break;
                        case 1403773248357:
                            cout << "\t" << q->getName() << " ID: " << q->getId() << endl;
                            checkQuantifier(q, 1403773248357, "MISSING_NAME", "", 1402488646221, {"TaskQuantifier"});
                            EXPECT_TRUE(dynamic_cast<const alica::ForallAgents*>(q) != 0) << "Wrong Type!" << endl;
                            break;
                        default:
                            EXPECT_TRUE(false);
                            cerr << "TEST_F(AlicaEngineTest, planParser) found a Quantifier not part of "
                                    "AttackPlan.pml!"
                                 << endl;
                            break;
                        }
                    }
                    break;
                case 1402489460694:
                    checkTransition(t, 1402489460694, "MISSING_NAME", "", 1402489462088, 1402489396914, 1402488646220, "Condition-Name-Shoot-Attack", "",
                            "Some nice comment!", "DefaultPlugin", true);
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a state not part of AttackPlan.pml!" << endl;
                    break;
                }
            }
            cout << "EntryPoints: " << endl;
            EXPECT_EQ(1, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit AttackPlan.pml EntryPoints size." << endl;
            for (const EntryPoint* ep : plan->getEntryPoints()) {
                cout << "\t" << ep->getName() << " ID: " << ep->getId() << endl;
                checkEntryPoint(ep, 1402488646221, "MISSING_NAME", "", false, 0, 2147483647, 1402488646220, 1225112227903, "DefaultTask");
            }
            cout << endl;
            break;
        case 1402488893641:
            checkPlan(plan, 1402488893641, "Defend", "", false, 0.1, 0, 2147483647);
            cout << "States: " << endl;
            EXPECT_EQ(4, plan->getStates().size()) << "Number of states didnt fit Defend.pml state size." << endl;
            for (const alica::State* s : plan->getStates()) {
                cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                switch (s->getId()) {
                case 1402488903549:
                    checkState(s, 1402488903549, "Tackle", "", {1402488956661, 1402489318663}, {1402488990761}, {1402488991762});
                    break;
                case 1402488910751:
                    EXPECT_TRUE(s->isFailureState()) << "Should be a FailureState" << endl;
                    checkState(s, 1402488910751, "GetGoal", "GetGoal", {}, {1402489071510}, {});
                    break;
                case 1402488959965:
                    checkState(s, 1402488959965, "GetBall", "", {}, {1402488991762}, {1402488990761, 1402489064693}, 1402488903550);
                    break;
                case 1402489037735:
                    checkState(s, 1402489037735, "TryToDefendGoal", "", {1402489564599}, {1402489064693}, {1402489071510});
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a state not part of Defend.pml!" << endl;
                    break;
                }
            }
            cout << "Transitions: " << endl;
            EXPECT_EQ(4, plan->getTransitions().size()) << "Number of Transitions didnt fit AttackPlan.pml EntryPoints size." << endl;
            for (const alica::Transition* t : plan->getTransitions()) {
                cout << "\t" << t->getName() << " ID: " << t->getId() << endl;
                switch (t->getId()) {
                case 1402488990761:
                    checkTransition(t, 1402488990761, "TackleToGetBall", "GetBallToTackle", 1402488991641, 1402488959965, 1402488903549, "MISSING_NAME", "", "",
                            "DefaultPlugin", true);
                    break;
                case 1402488991762:
                    checkTransition(t, 1402488991762, "TackleToGetBall", "TackleToGetBall", 1402488993122, 1402488903549, 1402488959965, "MISSING_NAME", "", "",
                            "DefaultPlugin", true);
                    break;
                case 1402489064693:
                    checkTransition(t, 1402489064693, "GetBallToTryToDefendGoal", "TESTESTETS", 1402489065962, 1402488959965, 1402489037735, "MISSING_NAME", "",
                            "", "DefaultPlugin", true);
                    break;
                case 1402489071510:
                    checkTransition(t, 1402489071510, "TryToDefendGoalToGetGoal", "TryToDefendGoalToGetGoal", 1402489073613, 1402489037735, 1402488910751,
                            "MISSING_NAME", "", "", "DefaultPlugin", true);
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a transition not part of Defend.pml!" << endl;
                    break;
                }
            }
            cout << "EntryPoints: " << endl;
            EXPECT_EQ(1, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit Tackle.pml EntryPoints size." << endl;
            for (const EntryPoint* ep : plan->getEntryPoints()) {
                cout << "\t" << ep->getName() << " ID: " << ep->getId() << endl;
                switch (ep->getId()) {
                case 1402488903550:
                    checkEntryPoint(ep, 1402488903550, "MISSING_NAME", "", false, 0, 2147483647, 1402488959965, 1225112227903, "DefaultTask");
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a EntryPoint not part of Defend.pml!" << endl;
                    break;
                }
            }
            cout << endl;
            break;
        case 1402488870347:
            checkPlan(plan, 1402488870347, "GoalPlan", "", false, 0.1, 0, 2147483647);
            checkPreCondition(plan->getPreCondition(), 1402489131988, "PreCondition", "Test PC", "", "DefaultPlugin", true);
            cout << "States: " << endl;
            EXPECT_EQ(3, plan->getStates().size()) << "Number of states didnt fit GoalPlan.pml state size." << endl;
            for (const alica::State* s : plan->getStates()) {
                cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                switch (s->getId()) {
                case 1402488881799:
                    checkState(s, 1402488881799, "Shoot", "", {}, {1402489205153}, {1402489173167}, 1402488881800);
                    break;
                case 1402489152217:
                    checkState(s, 1402489152217, "Miss", "", {}, {1402489173167}, {1402489205153, 1402489216617});
                    break;
                case 1402489192198: {
                    EXPECT_TRUE(s->isSuccessState()) << "Should be a SuccessState" << endl;
                    checkState(s, 1402489192198, "Scored", "", {}, {1402489216617}, {});
                    alica::TerminalState* terminalState = (alica::TerminalState*) s;
                    checkPostCondition(terminalState->getPostCondition(), 1402489620773, "MISSING_NAME", "Test POSTC", "", "DefaultPlugin");
                    break;
                }
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a state not part of GoalPlan.pml!" << endl;
                    break;
                }
            }
            cout << "Transitions: " << endl;
            EXPECT_EQ(3, plan->getTransitions().size()) << "Number of Transitions didnt fit AttackPlan.pml EntryPoints size." << endl;
            for (const alica::Transition* t : plan->getTransitions()) {
                cout << "\t" << t->getName() << " ID: " << t->getId() << endl;
                switch (t->getId()) {
                case 1402489173167:
                    checkTransition(
                            t, 1402489173167, "MISSING_NAME", "", 1402489174338, 1402488881799, 1402489152217, "MISSING_NAME", "", "", "DefaultPlugin", true);
                    break;
                case 1402489205153:
                    checkTransition(
                            t, 1402489205153, "MISSING_NAME", "", 1402489206278, 1402489152217, 1402488881799, "MISSING_NAME", "", "", "DefaultPlugin", true);
                    break;
                case 1402489216617:
                    checkTransition(
                            t, 1402489216617, "MISSING_NAME", "", 1402489218027, 1402489152217, 1402489192198, "MISSING_NAME", "", "", "DefaultPlugin", true);
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a transition not part of GoalPlan.pml!" << endl;
                    break;
                }
            }
            cout << "EntryPoints: " << endl;
            EXPECT_EQ(1, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit Tackle.pml EntryPoints size." << endl;
            for (const EntryPoint* ep : plan->getEntryPoints()) {
                cout << "\t" << ep->getName() << " ID: " << ep->getId() << endl;
                switch (ep->getId()) {
                case 1402488881800:
                    checkEntryPoint(ep, 1402488881800, "MISSING_NAME", "", false, 0, 2147483647, 1402488881799, 1225112227903, "DefaultTask");
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a EntryPoint not part of GoalPlan.pml!" << endl;
                    break;
                }
            }
            cout << endl;
            break;
        case 1402488437260:
            checkPlan(plan, 1402488437260, "MasterPlan", "comment", true, 0.1, 0, 2147483647);
            cout << "States: " << endl;
            EXPECT_EQ(5, plan->getStates().size()) << "Number of states didnt fit MasterPlan.pml state size." << endl;
            for (const alica::State* s : plan->getStates()) {
                cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                switch (s->getId()) {
                case 1402488437261:
                    checkState(s, 1402488437261, "Attack", "", {1402488866727}, {}, {1402488517667, 1409218318661}, 1402488437263);
                    break;
                case 1402488463437:
                    checkState(s, 1402488463437, "Defend", "", {1402488893641}, {1409218318661}, {});
                    break;
                case 1402488470615:
                    checkState(s, 1402488470615, "Goal", "", {1402488870347}, {1402488519757}, {1402488557864});
                    break;
                case 1402488477650:
                    checkState(s, 1402488477650, "MidField", "", {1402488712657, 1402488763903, 1402488770050}, {1402488517667}, {1402488519757});
                    break;
                case 1402488536570:
                    EXPECT_TRUE(s->isSuccessState()) << "Should be a successstate!" << endl;
                    checkState(s, 1402488536570, "SucGoalState", "", {}, {1402488557864}, {});
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a state not part of MasterPlan.pml!" << endl;
                    break;
                }
            }
            cout << "Transitions: " << endl;
            EXPECT_EQ(4, plan->getTransitions().size()) << "Number of Transitions didnt fit AttackPlan.pml EntryPoints size." << endl;
            for (const alica::Transition* t : plan->getTransitions()) {
                cout << "\t" << t->getName() << " ID: " << t->getId() << endl;
                switch (t->getId()) {
                case 1402488557864:
                    checkTransition(t, 1402488557864, "GoalToSucGoal", "GoalToSucGoal", 1402488558741, 1402488470615, 1402488536570, "MISSING_NAME", "", "",
                            "DefaultPlugin", true);
                    break;
                case 1402488517667:
                    checkTransition(t, 1402488517667, "AttackToGoal", "AttackToGoal", 1402488519140, 1402488437261, 1402488477650, "MISSING_NAME", "", "",
                            "DefaultPlugin", true);
                    break;
                case 1402488519757:
                    checkTransition(t, 1402488519757, "MidFieldToGoal", "MidFieldToGoal", 1402488520968, 1402488477650, 1402488470615, "MISSING_NAME", "", "",
                            "DefaultPlugin", true);
                    break;
                case 1409218318661:
                    checkTransition(t, 1409218318661, "AttackToDefend", "AttackToDefend", 1409218319990, 1402488437261, 1402488463437, "MISSING_NAME", "", "",
                            "DefaultPlugin", true);
                    break;
                default:
                    cout << t->getId() << "########" << endl;
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a transition not part of MasterPlan.pml!" << endl;
                    break;
                }
            }
            cout << "EntryPoints: " << endl;
            EXPECT_EQ(1, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit Tackle.pml EntryPoints size." << endl;
            for (const EntryPoint* ep : plan->getEntryPoints()) {
                cout << "\t" << ep->getName() << " ID: " << ep->getId() << endl;
                switch (ep->getId()) {
                case 1402488437263:
                    checkEntryPoint(ep, 1402488437263, "MISSING_NAME", "", false, 0, 2147483647, 1402488437261, 1225112227903, "DefaultTask");
                    break;
                case 1402488484084:
                    checkEntryPoint(ep, 1402488484084, "MISSING_NAME", "", false, 0, 2147483647, 1402488463437, 1402488486725, "DefendTask");
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a EntryPoint not part of MasterPlan.pml!" << endl;
                    break;
                }
            }
            cout << endl;
            break;
        case 1402488770050:
            checkPlan(plan, 1402488770050, "MidFieldPlayPlan", "", false, 0.1, 3, 2147483647);
            checkRuntimeCondition(plan->getRuntimeCondition(), 1402489260911, "NewRuntimeCondition", "Test RC", "", "DefaultPlugin");
            cout << "States: " << endl;
            EXPECT_EQ(5, plan->getStates().size()) << "Number of states didnt fit MidFieldPlayPlan.pml state size." << endl;
            for (const alica::State* s : plan->getStates()) {
                cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                switch (s->getId()) {
                case 1402488787818:
                    checkState(s, 1402488787818, "Wander", "", {1402488712657}, {}, {1402489257607, 1402489276995}, 1402488787819);
                    break;
                case 1402489237914:
                    checkState(s, 1402489237914, "Tackle", "", {1402489318663, 1402488866727, 1402488893641}, {1402489257607}, {});
                    break;
                case 1402489273401:
                    checkState(s, 1402489273401, "Sync", "", {1402488956661}, {1402489276995}, {});
                    break;
                case 1402500830885:
                    checkState(s, 1402500830885, "Kill", "", {1403773823508}, {}, {1402500843072}, 1402500828244);
                    break;
                case 1402500833246:
                    checkState(s, 1402500833246, "Shoot", "", {}, {1402500843072}, {});
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a state not part of "
                            "MidFieldPlayPlan.pml!"
                         << endl;
                    break;
                }
            }
            cout << "Transitions: " << endl;
            EXPECT_EQ(3, plan->getTransitions().size()) << "Number of Transitions didnt fit MidFieldPlayPlan.pml EntryPoints size." << endl;
            for (const alica::Transition* t : plan->getTransitions()) {
                cout << "\t" << t->getName() << " ID: " << t->getId() << endl;
                switch (t->getId()) {
                case 1402489257607:
                    checkTransition(
                            t, 1402489257607, "MISSING_NAME", "", 1402489258509, 1402488787818, 1402489237914, "MISSING_NAME", "", "", "DefaultPlugin", true);
                    break;
                case 1402489276995:
                    checkTransition(
                            t, 1402489276995, "MISSING_NAME", "", 1402489278408, 1402488787818, 1402489273401, "MISSING_NAME", "", "", "DefaultPlugin", true);
                    EXPECT_EQ(1402500865502, t->getSyncTransition()->getId()) << "Wrong synctransition ID!" << endl;
                    break;
                case 1402500843072:
                    checkTransition(
                            t, 1402500843072, "MISSING_NAME", "", 1402500844446, 1402500830885, 1402500833246, "MISSING_NAME", "", "", "DefaultPlugin", true);
                    checkSyncTransition(t->getSyncTransition(), 1402500865502, "SynChro", "", 30, 10000);

                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a transition not part of "
                            "MidFieldPlayPlan.pml!"
                         << endl;
                    break;
                }
            }
            cout << "EntryPoints: " << endl;
            EXPECT_EQ(2, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit MidFieldPlayPlan.pml EntryPoints size." << endl;
            for (const EntryPoint* ep : plan->getEntryPoints()) {
                cout << "\t" << ep->getName() << " ID: " << ep->getId() << endl;
                switch (ep->getId()) {
                case 1402488787819:
                    checkEntryPoint(ep, 1402488787819, "MISSING_NAME", "", true, 0, 2147483647, 1402488787818, 1225112227903, "DefaultTask");
                    break;
                case 1402500828244:
                    checkEntryPoint(ep, 1402500828244, "NewEntryPoint", "TestComment", false, 3, 5, 1402500830885, 1225112227903, "DefaultTask");
                    break;
                default:
                    EXPECT_TRUE(false);
                    cerr << "TEST_F(AlicaEngineTest, planParser) found a EntryPoint not part of "
                            "MidFieldPlayPlan.pml!"
                         << endl;
                    break;
                }
            }
            cout << endl;
            break;
        case 1402489318663:
            checkPlan(plan, 1402489318663, "Tackle", "", false, 0.1, 0, 2147483647);
            cout << "States: " << endl;
            EXPECT_EQ(1, plan->getStates().size()) << "Number of states didnt fit Tackle.pml state size." << endl;
            for (const alica::State* s : plan->getStates()) {
                cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                checkState(s, 1402489329141, "AttackOpp", "", {1402489366699}, {}, {}, 1402489329142);
            }
            cout << "EntryPoints: " << endl;
            EXPECT_EQ(1, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit Tackle.pml EntryPoints size." << endl;
            for (const EntryPoint* ep : plan->getEntryPoints()) {
                cout << "\t" << ep->getName() << " ID: " << ep->getId() << endl;
                checkEntryPoint(ep, 1402489329142, "MISSING_NAME", "", false, 0, 2147483647, 1402489329141, 1225112227903, "DefaultTask");
            }
            cout << endl;
            break;
        default:
            EXPECT_TRUE(false);
            cerr << "TEST_F(AlicaEngineTest, planParser) found an id not part of testplans but expect_true for all "
                    "ids failed!"
                 << endl;
            break;
        }
    }
}

TEST_F(AlicaEngineTest, planWriter)
{
    ASSERT_NO_SIGNAL

    const auto& plans = ae->getPlanRepository()->getPlans();
    PlanWriter pw = PlanWriter(ae, ae->getPlanRepository());
    for (const Plan* plan : plans) {
        cout << "AlicaEngineTest, planWriter: Writing Plan " << plan->getName() << endl;
        pw.saveSinglePlan(plan);
        string temp = supplementary::FileSystem::combinePaths(sc->getConfigPath(), "plans/tmp");
        temp = supplementary::FileSystem::combinePaths(temp, plan->getName() + string(".pml"));
        string test = exec((string("diff ") + plan->getFileName() + string(" ") + temp).c_str());
        EXPECT_EQ(0, test.size()) << "files are different! " << test << endl;
        std::remove(temp.c_str()); // delete the file after comparing it
    }
    cout << "AlicaEngineTest, planWriter: writing plans done." << endl;
}
