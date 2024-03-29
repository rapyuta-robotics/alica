using namespace std;

#include "test_alica.h"

#include "engine/AlicaClock.h"
#include "engine/BasicBehaviour.h"
#include "engine/DefaultUtilityFunction.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/AlicaElement.h"
#include "engine/model/Behaviour.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/ForallAgents.h"
#include "engine/model/Plan.h"
#include "engine/model/PostCondition.h"
#include "engine/model/PreCondition.h"
#include "engine/model/Quantifier.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/State.h"
#include "engine/model/Synchronisation.h"
#include "engine/model/Task.h"
#include "engine/model/TerminalState.h"
#include "engine/model/Transition.h"
#include "engine/model/TransitionCondition.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <iostream>
#include <list>

namespace alica
{
namespace
{

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
        EXPECT_EQ(absPlanIDs.size(), s->getConfAbstractPlanWrappers().size()) << "Number of abstractPlans didnt fit plans size." << endl;
        for (const ConfAbstractPlanWrapper* wrapper : s->getConfAbstractPlanWrappers()) {
            const alica::AbstractPlan* p = wrapper->getAbstractPlan();
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

    static void checkPlan(const alica::Plan* plan, long id, string name, string comment, double utilityThreshold, int minCardinality, int maxCardinality)
    {
        checkAlicaElement(plan, id, name, comment);
        EXPECT_EQ(utilityThreshold, plan->getUtilityThreshold()) << "Wrong utilityThreshold!" << endl;
        EXPECT_EQ(minCardinality, plan->getMinCardinality()) << "Wrong minCardinality!" << endl;
        EXPECT_EQ(maxCardinality, plan->getMaxCardinality()) << "Wrong maxCardinality!" << endl;
    }

    static void checkPreCondition(const alica::PreCondition* condition, long id, string name, string comment, string conString, bool enabled)
    {
        checkAlicaElement(condition, id, name, comment);
        EXPECT_STREQ(conString.c_str(), condition->getConditionString().c_str()) << "Wrong ConditionString!" << endl;
        EXPECT_EQ(enabled, condition->isEnabled()) << "Wrong enabled value!" << endl;
    }

    static void checkTransitionCondition(const alica::TransitionCondition* condition, long id, string name, string comment)
    {
        checkAlicaElement(condition, id, name, comment);
    }

    static void checkPostCondition(const alica::PostCondition* condition, long id, string name, string comment, string conString)
    {
        checkAlicaElement(condition, id, name, comment);
        EXPECT_STREQ(conString.c_str(), condition->getConditionString().c_str()) << "Wrong ConditionString!" << endl;
    }

    static void checkRuntimeCondition(const alica::RuntimeCondition* condition, long id, string name, string comment, string conString)
    {
        checkAlicaElement(condition, id, name, comment);
        EXPECT_STREQ(conString.c_str(), condition->getConditionString().c_str()) << "Wrong ConditionString!" << endl;
    }
    static void checkTransition(const alica::Transition* transition, long id, string name, string comment, long preConditionId, long inState, long outState,
            string preConName, string preConComment, string preConString, bool enabled)
    {
        checkAlicaElement(transition, id, name, comment);
        checkTransitionCondition(transition->getTransitionCondition(), preConditionId, preConName, preConComment);
        // checkPreCondition(transition->getPreCondition(), preConditionId, preConName, preConComment, preConString, enabled);

        EXPECT_EQ(inState, transition->getInState()->getId()) << "Unknown id for InState!" << endl;

        EXPECT_EQ(outState, transition->getOutState()->getId()) << "Unknown id for OutState!" << endl;
    }

    static void checkSynchronisation(const alica::Synchronisation* synchronisation, long id, string name, string comment, int talkTimeout, int syncTimeout)
    {
        checkAlicaElement(synchronisation, id, name, comment);
        EXPECT_EQ(alica::AlicaTime::milliseconds(talkTimeout), synchronisation->getTalkTimeOut()) << "Wrong talkTimeout!" << endl;
        EXPECT_EQ(alica::AlicaTime::milliseconds(syncTimeout), synchronisation->getSyncTimeOut()) << "Wrong syncTimeout!" << endl;
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

    void verifyParsing()
    {
        const auto& plans = ae->getPlanRepository().getPlans();

        cout << "Printing plans from Repository: " << endl;
        for (const alica::Plan* plan : plans) {
            cout << "--------- Next Plan: -------------" << endl;
            cout << "ID: " << plan->getId() << endl;
            cout << "Plan: " << plan->toString() << endl;
            EXPECT_TRUE(plan->getId() == 1402488634525 || plan->getId() == 1402488893641 || plan->getId() == 1402488870347 || plan->getId() == 1402488437260 ||
                        plan->getId() == 1402488770050 || plan->getId() == 1402489318663)
                    << "ID not part of testplans!" << endl;
            switch (plan->getId()) {
            case 1402488634525:
                checkPlan(plan, 1402488634525, "AttackPlan", "", 0.1, 0, 2147483647);
                cout << "States: " << endl;
                EXPECT_EQ(2u, plan->getStates().size()) << "Number of states didnt fit AttackPlan.pml state size." << endl;
                for (const alica::State* s : plan->getStates()) {
                    cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                    switch (s->getId()) {
                    case 1402488646220:
                        checkState(s, 1402488646220, "Attack", "", {1402489351885, 1402489318663}, {1402489460694}, {1402489459382}, 1402488646221);
                        break;
                    case 1402489396914:
                        checkState(s, 1402489396914, "Shoot", "", {1402488848841}, {1402489459382}, {1402489460694});
                        break;
                    default:
                        EXPECT_TRUE(false);
                        cerr << "TEST_F(AlicaEngineTest, planParser) found a state not part of AttackPlan.pml!" << endl;
                        break;
                    }
                }
                cout << "Transitions: " << endl;
                EXPECT_EQ(2u, plan->getTransitions().size()) << "Number of Transitions didnt fit AttackPlan.pml EntryPoints size." << endl;
                for (const alica::Transition* t : plan->getTransitions()) {
                    cout << "\t" << t->getName() << " ID: " << t->getId() << endl;
                    switch (t->getId()) {
                    case 1402489459382:
                        checkTransition(
                                t, 1402489459382, "MISSING_NAME", "", 1678986049909129132, 1402488646220, 1402489396914, "AlwaysFalseCondition", "", "", true);
                        cout << "Quantifiers: " << endl;
                        // for (const alica::Quantifier* q : t->getPreCondition()->getQuantifiers()) {
                        //     switch (q->getId()) {
                        //     case 1403773214317:
                        //         cout << "\t" << q->getName() << " ID: " << q->getId() << endl;
                        //         checkQuantifier(q, 1403773214317, "MISSING_NAME", "", 1402488634525, {"X", "Y"});
                        //         EXPECT_TRUE(dynamic_cast<const alica::ForallAgents*>(q) != 0) << "Wrong Type!" << endl;
                        //         break;
                        //     case 1403773224776:
                        //         cout << "\t" << q->getName() << " ID: " << q->getId() << endl;
                        //         checkQuantifier(q, 1403773224776, "MISSING_NAME", "", 1402488646220, {"A", "B"});
                        //         EXPECT_TRUE(dynamic_cast<const alica::ForallAgents*>(q) != 0) << "Wrong Type!" << endl;
                        //         break;
                        //     case 1403773234841:
                        //         cout << "\t" << q->getName() << " ID: " << q->getId() << endl;
                        //         checkQuantifier(q, 1403773234841, "MISSING_NAME", "", 1402489396914, {"another one"});
                        //         EXPECT_TRUE(dynamic_cast<const alica::ForallAgents*>(q) != 0) << "Wrong Type!" << endl;
                        //         break;
                        //     case 1403773248357:
                        //         cout << "\t" << q->getName() << " ID: " << q->getId() << endl;
                        //         checkQuantifier(q, 1403773248357, "MISSING_NAME", "", 1402488646221, {"TaskQuantifier"});
                        //         EXPECT_TRUE(dynamic_cast<const alica::ForallAgents*>(q) != 0) << "Wrong Type!" << endl;
                        //         break;
                        //     default:
                        //         EXPECT_TRUE(false);
                        //         cerr << "TEST_F(AlicaEngineTest, planParser) found a Quantifier not part of "
                        //                 "AttackPlan.pml!"
                        //              << endl;
                        //         break;
                        //     }
                        // }
                        break;
                    case 1402489460694:
                        checkTransition(t, 1402489460694, "MISSING_NAME", "", 1678986049909129132, 1402489396914, 1402488646220, "AlwaysFalseCondition", "",
                                "Some nice comment!", true);
                        break;
                    default:
                        EXPECT_TRUE(false);
                        cerr << "TEST_F(AlicaEngineTest, planParser) found a state not part of AttackPlan.pml!" << endl;
                        break;
                    }
                }
                cout << "EntryPoints: " << endl;
                EXPECT_EQ(1u, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit AttackPlan.pml EntryPoints size." << endl;
                for (const alica::EntryPoint* ep : plan->getEntryPoints()) {
                    cout << "\t" << ep->getName() << " ID: " << ep->getId() << endl;
                    checkEntryPoint(ep, 1402488646221, "MISSING_NAME", "", false, 0, 2147483647, 1402488646220, 1225112227903, "DefaultTask");
                }
                cout << endl;
                break;
            case 1402488893641:
                checkPlan(plan, 1402488893641, "Defend", "", 0.1, 0, 2147483647);
                cout << "States: " << endl;
                EXPECT_EQ(4u, plan->getStates().size()) << "Number of states didnt fit Defend.pml state size." << endl;
                for (const alica::State* s : plan->getStates()) {
                    cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                    switch (s->getId()) {
                    case 1402488903549:
                        checkState(s, 1402488903549, "Tackle", "", {1402488939130, 1402489318663}, {1402488990761}, {1402488991762});
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
                EXPECT_EQ(4u, plan->getTransitions().size()) << "Number of Transitions didnt fit AttackPlan.pml EntryPoints size." << endl;
                for (const alica::Transition* t : plan->getTransitions()) {
                    cout << "\t" << t->getName() << " ID: " << t->getId() << endl;
                    switch (t->getId()) {
                    case 1402488990761:
                        checkTransition(t, 1402488990761, "TackleToGetBall", "GetBallToTackle", 1678986049909129132, 1402488959965, 1402488903549,
                                "AlwaysFalseCondition", "", "", true);
                        break;
                    case 1402488991762:
                        checkTransition(t, 1402488991762, "TackleToGetBall", "TackleToGetBall", 1678986049909129132, 1402488903549, 1402488959965,
                                "AlwaysFalseCondition", "", "", true);
                        break;
                    case 1402489064693:
                        checkTransition(t, 1402489064693, "GetBallToTryToDefendGoal", "TESTESTETS", 1678986049909129132, 1402488959965, 1402489037735,
                                "AlwaysFalseCondition", "", "", true);
                        break;
                    case 1402489071510:
                        checkTransition(t, 1402489071510, "TryToDefendGoalToGetGoal", "TryToDefendGoalToGetGoal", 1678986049909129132, 1402489037735,
                                1402488910751, "AlwaysFalseCondition", "", "", true);
                        break;
                    default:
                        EXPECT_TRUE(false);
                        cerr << "TEST_F(AlicaEngineTest, planParser) found a transition not part of Defend.pml!" << endl;
                        break;
                    }
                }
                cout << "EntryPoints: " << endl;
                EXPECT_EQ(1u, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit Tackle.pml EntryPoints size." << endl;
                for (const alica::EntryPoint* ep : plan->getEntryPoints()) {
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
                checkPlan(plan, 1402488870347, "GoalPlan", "", 0.1, 0, 2147483647);
                checkPreCondition(plan->getPreCondition(), 1402489131988, "GoalPlanPreCondition", "Test PC", "", true);
                cout << "States: " << endl;
                EXPECT_EQ(3u, plan->getStates().size()) << "Number of states didnt fit GoalPlan.pml state size." << endl;
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
                        checkPostCondition(terminalState->getPostCondition(), 1402489620773, "MISSING_NAME", "Test POSTC", "");
                        break;
                    }
                    default:
                        EXPECT_TRUE(false);
                        cerr << "TEST_F(AlicaEngineTest, planParser) found a state not part of GoalPlan.pml!" << endl;
                        break;
                    }
                }
                cout << "Transitions: " << endl;
                EXPECT_EQ(3u, plan->getTransitions().size()) << "Number of Transitions didnt fit AttackPlan.pml EntryPoints size." << endl;
                for (const alica::Transition* t : plan->getTransitions()) {
                    cout << "\t" << t->getName() << " ID: " << t->getId() << endl;
                    switch (t->getId()) {
                    case 1402489173167:
                        checkTransition(
                                t, 1402489173167, "MISSING_NAME", "", 1678986049909129132, 1402488881799, 1402489152217, "AlwaysFalseCondition", "", "", true);
                        break;
                    case 1402489205153:
                        checkTransition(
                                t, 1402489205153, "MISSING_NAME", "", 1678986049909129132, 1402489152217, 1402488881799, "AlwaysFalseCondition", "", "", true);
                        break;
                    case 1402489216617:
                        checkTransition(
                                t, 1402489216617, "MISSING_NAME", "", 1678986049909129132, 1402489152217, 1402489192198, "AlwaysFalseCondition", "", "", true);
                        break;
                    default:
                        EXPECT_TRUE(false);
                        cerr << "TEST_F(AlicaEngineTest, planParser) found a transition not part of GoalPlan.pml!" << endl;
                        break;
                    }
                }
                cout << "EntryPoints: " << endl;
                EXPECT_EQ(1u, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit Tackle.pml EntryPoints size." << endl;
                for (const alica::EntryPoint* ep : plan->getEntryPoints()) {
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
                checkPlan(plan, 1402488437260, "MasterPlan", "comment", 0.1, 0, 2147483647);
                cout << "States: " << endl;
                EXPECT_EQ(5u, plan->getStates().size()) << "Number of states didnt fit MasterPlan.pml state size." << endl;
                for (const alica::State* s : plan->getStates()) {
                    cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                    switch (s->getId()) {
                    case 1402488437261:
                        checkState(s, 1402488437261, "Attack", "", {1402488848841}, {}, {1402488517667, 1409218318661}, 1402488437263);
                        break;
                    case 1402488463437:
                        checkState(s, 1402488463437, "Defend", "", {1402488893641}, {1409218318661}, {});
                        break;
                    case 1402488470615:
                        checkState(s, 1402488470615, "Goal", "", {1402488870347}, {1402488519757}, {1402488557864});
                        break;
                    case 1402488477650:
                        checkState(s, 1402488477650, "MidField", "", {1402488696205, 1402488730695, 1402488770050}, {1402488517667}, {1402488519757});
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
                EXPECT_EQ(4u, plan->getTransitions().size()) << "Number of Transitions didnt fit AttackPlan.pml EntryPoints size." << endl;
                for (const alica::Transition* t : plan->getTransitions()) {
                    cout << "\t" << t->getName() << " ID: " << t->getId() << endl;
                    switch (t->getId()) {
                    case 1402488557864:
                        checkTransition(t, 1402488557864, "GoalToSucGoal", "GoalToSucGoal", 1678986049909129132, 1402488470615, 1402488536570,
                                "AlwaysFalseCondition", "", "", true);
                        break;
                    case 1402488517667:
                        checkTransition(t, 1402488517667, "AttackToGoal", "AttackToGoal", 1678986049909129132, 1402488437261, 1402488477650,
                                "AlwaysFalseCondition", "", "", true);
                        break;
                    case 1402488519757:
                        checkTransition(t, 1402488519757, "MidFieldToGoal", "MidFieldToGoal", 1678986049909129132, 1402488477650, 1402488470615,
                                "AlwaysFalseCondition", "", "", true);
                        break;
                    case 1409218318661:
                        checkTransition(t, 1409218318661, "AttackToDefend", "AttackToDefend", 1678986049909129132, 1402488437261, 1402488463437,
                                "AlwaysFalseCondition", "", "", true);
                        break;
                    default:
                        cout << t->getId() << "########" << endl;
                        EXPECT_TRUE(false);
                        cerr << "TEST_F(AlicaEngineTest, planParser) found a transition not part of MasterPlan.pml!" << endl;
                        break;
                    }
                }
                cout << "EntryPoints: " << endl;
                EXPECT_EQ(1u, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit Tackle.pml EntryPoints size." << endl;
                for (const alica::EntryPoint* ep : plan->getEntryPoints()) {
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
                checkPlan(plan, 1402488770050, "MidFieldPlayPlan", "", 0.1, 3, 2147483647);
                checkRuntimeCondition(plan->getRuntimeCondition(), 1402489260911, "MidFieldPlayPlanRuntimeCondition", "Test RC", "");
                cout << "States: " << endl;
                EXPECT_EQ(5u, plan->getStates().size()) << "Number of states didnt fit MidFieldPlayPlan.pml state size." << endl;
                for (const alica::State* s : plan->getStates()) {
                    cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                    switch (s->getId()) {
                    case 1402488787818:
                        checkState(s, 1402488787818, "Wander", "", {1402488696205}, {}, {1402489257607, 1402489276995}, 1402488787819);
                        break;
                    case 1402489237914:
                        checkState(s, 1402489237914, "Tackle", "", {1402489318663, 1402488848841, 1402488893641}, {1402489257607}, {});
                        break;
                    case 1402489273401:
                        checkState(s, 1402489273401, "Sync", "", {1402488939130}, {1402489276995}, {});
                        break;
                    case 1402500830885:
                        checkState(s, 1402500830885, "Kill", "", {}, {}, {1402500843072}, 1402500828244);
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
                EXPECT_EQ(3u, plan->getTransitions().size()) << "Number of Transitions didnt fit MidFieldPlayPlan.pml EntryPoints size." << endl;
                for (const alica::Transition* t : plan->getTransitions()) {
                    cout << "\t" << t->getName() << " ID: " << t->getId() << endl;
                    switch (t->getId()) {
                    case 1402489257607:
                        checkTransition(
                                t, 1402489257607, "MISSING_NAME", "", 1678986049909129132, 1402488787818, 1402489237914, "AlwaysFalseCondition", "", "", true);
                        break;
                    case 1402489276995:
                        checkTransition(
                                t, 1402489276995, "MISSING_NAME", "", 1678986049909129132, 1402488787818, 1402489273401, "AlwaysFalseCondition", "", "", true);
                        EXPECT_EQ(1402500865502, t->getSynchronisation()->getId()) << "Wrong synctransition ID!" << endl;
                        break;
                    case 1402500843072:
                        checkTransition(
                                t, 1402500843072, "MISSING_NAME", "", 1678986049909129132, 1402500830885, 1402500833246, "AlwaysFalseCondition", "", "", true);
                        checkSynchronisation(t->getSynchronisation(), 1402500865502, "SynChro", "", 30, 10000);

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
                EXPECT_EQ(2u, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit MidFieldPlayPlan.pml EntryPoints size." << endl;
                for (const alica::EntryPoint* ep : plan->getEntryPoints()) {
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
                checkPlan(plan, 1402489318663, "Tackle", "", 0.1, 0, 2147483647);
                cout << "States: " << endl;
                EXPECT_EQ(1u, plan->getStates().size()) << "Number of states didnt fit Tackle.pml state size." << endl;
                for (const alica::State* s : plan->getStates()) {
                    cout << "\t" << s->getName() << " ID: " << s->getId() << endl;
                    checkState(s, 1402489329141, "AttackOpp", "", {1402489351885}, {}, {}, 1402489329142);
                }
                cout << "EntryPoints: " << endl;
                EXPECT_EQ(1u, plan->getEntryPoints().size()) << "Number of EntryPoints didnt fit Tackle.pml EntryPoints size." << endl;
                for (const alica::EntryPoint* ep : plan->getEntryPoints()) {
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
};

class MultiConfigFolderTest : public AlicaEngineTest
{
protected:
    std::vector<std::string> getTestFolderPaths() const override
    {
        std::string path;
#if defined(THIS_PACKAGE_DIR)
        path = std::string{THIS_PACKAGE_DIR} + "/etc/";
#endif

        // Use the sub-folders in the etc folder as separate config folders
        return std::vector<std::string>{path + "/hairy", path + "/nase", path + "/plans", path + "/roles", path + "/tasks", path + "/behaviours",
                path + "/conditions", path + "/configurations"};
    }
};

/**
 * Tests the plan parser with some nice plans & a single config folder
 */
TEST_F(AlicaEngineTest, planParser)
{
    ASSERT_NO_SIGNAL
    verifyParsing();
}

TEST_F(MultiConfigFolderTest, multiFolderPlanParsingTest)
{
    // Tests the plan parsing with the same plans as the planParser test, but with multiple config folders instead of one
    // The config folders are an aggregate of the folders that form the etc folder
    ASSERT_NO_SIGNAL
    verifyParsing();
}

TEST_F(AlicaEngineTest, planWriter)
{
    ASSERT_NO_SIGNAL
    cout << "AlicaEngineTest, planWriter: Develop plan writer for JSON first." << endl;
    //    const auto& plans = ae->getPlanRepository().getPlans();
    //    alica::PlanWriter pw = alica::PlanWriter(ae);
    //    for (const alica::Plan* plan : plans) {
    //        cout << "AlicaEngineTest, planWriter: Writing Plan " << plan->getName() << endl;
    //        pw.saveSinglePlan(plan);
    //        essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    //        string temp = essentials::FileSystem::combinePaths(sc.getConfigPath(), "plans/tmp");
    //        temp = essentials::FileSystem::combinePaths(temp, plan->getName() + string(".pml"));
    //        string test = exec((string("diff ") + plan->getFileName() + string(" ") + temp).c_str());
    //        EXPECT_EQ(0, test.size()) << "files are different! " << test << endl;
    //        std::remove(temp.c_str()); // delete the file after comparing it
    //    }
    //    cout << "AlicaEngineTest, planWriter: writing plans done." << endl;
}
} // namespace
} // namespace alica
