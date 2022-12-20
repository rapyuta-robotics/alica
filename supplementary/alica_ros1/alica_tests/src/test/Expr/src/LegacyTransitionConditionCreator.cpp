#include "alica_tests/LegacyTransitionConditionCreator.h"

#include <alica_tests/AdjacentSuccessMasterPlan3254486013443203397.h>
#include <alica_tests/AdjacentSuccessSubPlan1682631238618360548.h>
#include <alica_tests/AttackPlan1402488634525.h>
#include <alica_tests/Authority/AuthorityTest1414403413451.h>
#include <alica_tests/Authority/AuthorityTestMaster1414403396328.h>
#include <alica_tests/BackForth1529456584982.h>
#include <alica_tests/BehaviorSuccessSpamMaster1522377375148.h>
#include <alica_tests/BehaviourTriggerTestPlan1428508768572.h>
#include <alica_tests/Configurations/ConfigurationTestPlan1588060981661.h>
#include <alica_tests/Configurations/ReadConfInPlantype1588061801734.h>
#include <alica_tests/Configurations/ReadConfigurationPlan1588061334567.h>
#include <alica_tests/ConstraintTestMaster1414068495566.h>
#include <alica_tests/ConstraintTestPlan1414068524245.h>
#include <alica_tests/Defend1402488893641.h>
#include <alica_tests/ExecuteBehaviourInSubPlan3172561495666303184.h>
#include <alica_tests/FailsOnOne1530069246103.h>
#include <alica_tests/FailureHandlingMaster4150733089768927549.h>
#include <alica_tests/FailurePlan631515556091266493.h>
#include <alica_tests/FrequencyTestPlan1626848999740.h>
#include <alica_tests/GoalPlan1402488870347.h>
#include <alica_tests/HandleFailExplicit1530004915640.h>
#include <alica_tests/HandleFailExplicitMaster1530004940652.h>
#include <alica_tests/MasterPlan1402488437260.h>
#include <alica_tests/MasterPlanTaskAssignment1407152758497.h>
#include <alica_tests/MasterPlanTestConditionPlanType1418042656594.h>
#include <alica_tests/MasterSyncTransition1418825395939.h>
#include <alica_tests/MidFieldPlayPlan1402488770050.h>
#include <alica_tests/MultiAgentTestMaster1413200842973.h>
#include <alica_tests/MultiAgentTestPlan1413200862180.h>
#include <alica_tests/OrderedSchedulingTestPlan1629895582410.h>
#include <alica_tests/OtherPlan1418042819203.h>
#include <alica_tests/PlanA1629895837159.h>
#include <alica_tests/PlanAA1629895864090.h>
#include <alica_tests/PlanB1629895853508.h>
#include <alica_tests/PlanBA1629895873188.h>
#include <alica_tests/PlanFive1407153703092.h>
#include <alica_tests/PlanFour1407153683051.h>
#include <alica_tests/PlanOne1407153611768.h>
#include <alica_tests/PlanThree1407153663917.h>
#include <alica_tests/PlanTwo1407153645238.h>
#include <alica_tests/PreConditionPlan1418042796751.h>
#include <alica_tests/RealMasterPlanForSyncTest1418902217839.h>
#include <alica_tests/RuntimeConditionPlan1418042806575.h>
#include <alica_tests/SchedulingTestMasterPlan1613378382024.h>
#include <alica_tests/SchedulingTestPlan11613378406860.h>
#include <alica_tests/SchedulingTestPlan21613378423610.h>
#include <alica_tests/SchedulingTestPlan31613378433623.h>
#include <alica_tests/SchedulingTestSequencePlan11614963946725.h>
#include <alica_tests/SchedulingTestSequenceSubPlan11614964379654.h>
#include <alica_tests/SchedulingTestSequenceSubPlan21614964444419.h>
#include <alica_tests/SchedulingTestSequenceSubPlan31614964478264.h>
#include <alica_tests/SimpleTestPlan1412252439925.h>
#include <alica_tests/Tackle1402489318663.h>
#include <alica_tests/TestInheritBlackboard1692837668719979400.h>
#include <alica_tests/TestInheritBlackboardMaster1179066429431332056.h>
#include <alica_tests/TestParameterPassing1692837668719979457.h>
#include <alica_tests/TestParameterPassingMaster1179066429431332055.h>
#include <alica_tests/TestTracingMasterPlan691392966514374878.h>
#include <alica_tests/TestTracingSubPlan1482512794732634139.h>
#include <engine/IAlicaWorldModel.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

namespace alica
{

LegacyTransitionConditionCreator::LegacyTransitionConditionCreator() {}

LegacyTransitionConditionCreator::~LegacyTransitionConditionCreator() {}

std::function<bool(const Blackboard*, const RunningPlan*, const IAlicaWorldModel*)> LegacyTransitionConditionCreator::createConditions(
        int64_t conditionId, TransitionConditionContext& context)
{
    int64_t preConditionId = context.preConditionId;
    switch (preConditionId) {
    case 1588069612661: {
        PreCondition1588069612661 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1588069615553: {
        PreCondition1588069615553 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 2038762164340314344: {
        PreCondition2038762164340314344 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 4351457352348187886: {
        PreCondition4351457352348187886 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1522377944921: {
        PreCondition1522377944921 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1522377946607: {
        PreCondition1522377946607 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402489460549: {
        PreCondition1402489460549 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402489462088: {
        PreCondition1402489462088 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402488519140: {
        PreCondition1402488519140 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402488520968: {
        PreCondition1402488520968 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402488558741: {
        PreCondition1402488558741 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1409218319990: {
        PreCondition1409218319990 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402489258509: {
        PreCondition1402489258509 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402489278408: {
        PreCondition1402489278408 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402500844446: {
        PreCondition1402500844446 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1414403842622: {
        PreCondition1414403842622 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1588253347213: {
        PreCondition1588253347213 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1588246141557: {
        PreCondition1588246141557 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1588246144841: {
        PreCondition1588246144841 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1529456610600: {
        PreCondition1529456610600 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 2529456610600: {
        PreCondition2529456610600 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1840401110297459509: {
        PreCondition1840401110297459509 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1530004993493: {
        PreCondition1530004993493 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1530004994611: {
        PreCondition1530004994611 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1532424093178: {
        PreCondition1532424093178 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1532424113475: {
        PreCondition1532424113475 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1418825427317: {
        PreCondition1418825427317 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1418825428924: {
        PreCondition1418825428924 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1413201227586: {
        PreCondition1413201227586 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1413201389955: {
        PreCondition1413201389955 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1529456610697: {
        PreCondition1529456610697 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1529456611916: {
        PreCondition1529456611916 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 488794245455049811: {
        PreCondition488794245455049811 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1614964566531: {
        PreCondition1614964566531 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1614964572495: {
        PreCondition1614964572495 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1614964575553: {
        PreCondition1614964575553 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1614964578016: {
        PreCondition1614964578016 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1943478533524176732: {
        PreCondition1943478533524176732 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402489174338: {
        PreCondition1402489174338 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402489206278: {
        PreCondition1402489206278 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402489218027: {
        PreCondition1402489218027 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402488991641: {
        PreCondition1402488991641 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402488993122: {
        PreCondition1402488993122 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402489065962: {
        PreCondition1402489065962 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1402489073613: {
        PreCondition1402489073613 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 597347780541336226: {
        PreCondition597347780541336226 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1067314038887345208: {
        PreCondition1067314038887345208 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1614960055821: {
        PreCondition1614960055821 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1614960063843: {
        PreCondition1614960063843 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 289358204208851392: {
        PreCondition289358204208851392 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 807250359520655888: {
        PreCondition807250359520655888 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1613530643882: {
        PreCondition1613530643882 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1615797316171: {
        PreCondition1615797316171 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1615797327077: {
        PreCondition1615797327077 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1615797365364: {
        PreCondition1615797365364 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1629895598471: {
        PreCondition1629895598471 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1629895607018: {
        PreCondition1629895607018 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 3213510506830850443: {
        PreCondition3213510506830850443 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 68542020926196536: {
        PreCondition68542020926196536 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 4165333637052704488: {
        PreCondition4165333637052704488 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 61978004585920576: {
        PreCondition61978004585920576 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1629895758612: {
        PreCondition1629895758612 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1629895768182: {
        PreCondition1629895768182 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1412761926856: {
        PreCondition1412761926856 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1418042683692: {
        PreCondition1418042683692 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1413201052549: {
        PreCondition1413201052549 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1413201367990: {
        PreCondition1413201367990 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    case 1413201370590: {
        PreCondition1413201370590 preCondition;
        return [preCondition](const Blackboard* bb, const RunningPlan* rp, const IAlicaWorldModel* wm) mutable {
            // Create shared ptr for API compatibility, use noop deleter to prevent RunningPlan deletion
            std::shared_ptr<RunningPlan> temp(const_cast<RunningPlan*>(rp), [](RunningPlan* p) { /*Noop deleter*/ });
            return preCondition.evaluate(temp, wm);
        };
    }
    default:
        std::cerr << "LegacyTransitionConditionCreator: Unknown condition id requested: " << preConditionId << std::endl;
        throw new std::exception();
        break;
    }
}
} /* namespace alica */