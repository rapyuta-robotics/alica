package de.unikassel.vs.alica.generator.cpp

import de.unikassel.vs.alica.generator.IConstraintCodeGenerator
import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour
import de.unikassel.vs.alica.planDesigner.alicamodel.Condition
import de.unikassel.vs.alica.planDesigner.alicamodel.Plan
import de.unikassel.vs.alica.planDesigner.alicamodel.ConfAbstractPlanWrapper
import de.unikassel.vs.alica.planDesigner.alicamodel.PostCondition
import de.unikassel.vs.alica.planDesigner.alicamodel.PreCondition
import de.unikassel.vs.alica.planDesigner.alicamodel.RuntimeCondition
import java.util.List
import java.util.Map
import de.unikassel.vs.alica.planDesigner.alicamodel.Transition
import de.unikassel.vs.alica.planDesigner.alicamodel.EntryPoint
import de.unikassel.vs.alica.planDesigner.alicamodel.State
import de.unikassel.vs.alica.planDesigner.alicamodel.TerminalState
import de.unikassel.vs.alica.planDesigner.alicamodel.Variable

class XtendTemplates {

    public Map<String, String> protectedRegions;

    public def void setProtectedRegions (Map<String, String> regions) {
        protectedRegions = regions;
    }

        def String planCreatorHeader()'''
    #pragma once
    #include <engine/IPlanCreator.h>

    #include <memory>
    #include <iostream>

    namespace alica
    {

        class BasicPlan;

        class PlanCreator : public IPlanCreator
        {
            public:
                PlanCreator();
                virtual ~PlanCreator();
                std::unique_ptr<BasicPlan> createPlan(int64_t planId, PlanContext& context) override;
        };

    } /* namespace alica */
    '''

    def String behaviourCreatorHeader()'''
#pragma once
#include <engine/IBehaviourCreator.h>

#include <memory>
#include <iostream>

namespace alica
{

    class BasicBehaviour;

    class BehaviourCreator : public IBehaviourCreator
    {
        public:
            BehaviourCreator();
            virtual ~BehaviourCreator();
            virtual std::unique_ptr<BasicBehaviour> createBehaviour(int64_t behaviourId, BehaviourContext& context) override;
    };

} /* namespace alica */
'''

def String planCreatorSource(List<Plan> plans, String pkgName)'''
#include <«pkgName»/PlanCreator.h>
#include "engine/BasicPlan.h"
«FOR plan : plans»
«IF (plan.relativeDirectory == null || plan.relativeDirectory.isEmpty)»
#include <«pkgName»/«plan.name»«plan.id».h>
«ELSE»
#include <«pkgName»/«plan.relativeDirectory»/«plan.name»«plan.id».h>
«ENDIF»
«ENDFOR»

namespace alica
{

    PlanCreator::PlanCreator()
    {
    }

    PlanCreator::~PlanCreator()
    {
    }

    std::unique_ptr<BasicPlan> PlanCreator::createPlan(int64_t planId, PlanContext& context)
    {
        switch(planId)
        {
            «FOR plan : plans»
                case «plan.id»:
                return std::make_unique<«plan.name»«plan.id»>(context);
                break;
            «ENDFOR»
            default:
            std::cerr << "PlanCreator: Unknown plan requested: " << planId << std::endl;
            throw new std::exception();
            break;
        }
    }
}
'''

    def String behaviourCreatorSource(List<Behaviour> behaviours, String pkgName)'''
#include <«pkgName»/BehaviourCreator.h>
#include "engine/BasicBehaviour.h"
«FOR beh : behaviours»
«IF (beh.relativeDirectory == null || beh.relativeDirectory.isEmpty)»
#include <«pkgName»/«beh.name».h>
«ELSE»
#include <«pkgName»/«beh.relativeDirectory»/«beh.name».h>
«ENDIF»
«ENDFOR»

namespace alica
{

    BehaviourCreator::BehaviourCreator()
    {
    }

    BehaviourCreator::~BehaviourCreator()
    {
    }

    std::unique_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(int64_t behaviourId, BehaviourContext& context)
    {
        switch(behaviourId)
        {
            «FOR beh : behaviours»
                case «beh.id»:
                return std::make_unique<«beh.name»>(context);
                break;
            «ENDFOR»
            default:
            std::cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourId << std::endl;
            throw new std::exception();
            break;
        }
    }
}
'''

    def String behaviourHeader(Behaviour behaviour, String pkgName)'''
#pragma once

#include <«pkgName»/DomainBehaviour.h>
/*PROTECTED REGION ID(inc«behaviour.id») ENABLED START*/
«IF (protectedRegions.containsKey("inc" + behaviour.id))»
«protectedRegions.get("inc" + behaviour.id)»
«ELSE»
//Add additional includes here
«ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    class «behaviour.name» : public DomainBehaviour
    {
        public:
            «behaviour.name»(BehaviourContext& context);
            virtual ~«behaviour.name»();
            virtual void run(void* msg);
            /*PROTECTED REGION ID(pub«behaviour.id») ENABLED START*/
            «IF (protectedRegions.containsKey("pub" + behaviour.id))»
«protectedRegions.get("pub" + behaviour.id)»
            «ELSE»
            //Add additional protected methods here
            «ENDIF»
            /*PROTECTED REGION END*/
        protected:
            virtual void initialiseParameters();
            /*PROTECTED REGION ID(pro«behaviour.id») ENABLED START*/
            «IF (protectedRegions.containsKey("pro" + behaviour.id))»
«protectedRegions.get("pro" + behaviour.id)»
            «ELSE»
            //Add additional protected methods here
            «ENDIF»
            /*PROTECTED REGION END*/
        private:
        /*PROTECTED REGION ID(prv«behaviour.id») ENABLED START*/
            «IF (protectedRegions.containsKey("prv" + behaviour.id))»
«protectedRegions.get("prv" + behaviour.id)»
            «ELSE»
            //Add additional private methods here
            «ENDIF»
        /*PROTECTED REGION END*/
    };
} /* namespace alica */
'''

    def String behaviourSource(Behaviour behaviour, String pkgName) '''
«IF (behaviour.relativeDirectory == null || behaviour.relativeDirectory.isEmpty)»
#include <«pkgName»/«behaviour.name».h>
«ELSE»
#include <«pkgName»/«behaviour.relativeDirectory»/«behaviour.name».h>
«ENDIF»
#include <memory>

/*PROTECTED REGION ID(inccpp«behaviour.id») ENABLED START*/
    «IF (protectedRegions.containsKey("inccpp" + behaviour.id))»
«protectedRegions.get("inccpp" + behaviour.id)»
    «ELSE»
        //Add additional includes here
    «ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    /*PROTECTED REGION ID(staticVars«behaviour.id») ENABLED START*/
    «IF (protectedRegions.containsKey("staticVars" + behaviour.id))»
«protectedRegions.get("staticVars" + behaviour.id)»
    «ELSE»
        //initialise static variables here
    «ENDIF»
    /*PROTECTED REGION END*/

    «behaviour.name»::«behaviour.name»(BehaviourContext& context) : DomainBehaviour(context)
    {
        /*PROTECTED REGION ID(con«behaviour.id») ENABLED START*/
        «IF (protectedRegions.containsKey("con" + behaviour.id))»
«protectedRegions.get("con" + behaviour.id)»
        «ELSE»
            //Add additional options here
        «ENDIF»
        /*PROTECTED REGION END*/

    }
    «behaviour.name»::~«behaviour.name»()
    {
        /*PROTECTED REGION ID(dcon«behaviour.id») ENABLED START*/
        «IF (protectedRegions.containsKey("dcon" + behaviour.id))»
«protectedRegions.get("dcon" + behaviour.id)»
        «ELSE»
            //Add additional options here
        «ENDIF»
        /*PROTECTED REGION END*/

    }
    void «behaviour.name»::run(void* msg)
    {
        /*PROTECTED REGION ID(run«behaviour.id») ENABLED START*/
        «IF (protectedRegions.containsKey("run" + behaviour.id))»
«protectedRegions.get("run" + behaviour.id)»
        «ELSE»
            //Add additional options here
        «ENDIF»
        /*PROTECTED REGION END*/

    }
    void «behaviour.name»::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters«behaviour.id») ENABLED START*/
        «IF (protectedRegions.containsKey("initialiseParameters" + behaviour.id))»
«protectedRegions.get("initialiseParameters" + behaviour.id)»
        «ELSE»
            //Add additional options here
        «ENDIF»

        /*PROTECTED REGION END*/

    }
    /*PROTECTED REGION ID(methods«behaviour.id») ENABLED START*/
        «IF (protectedRegions.containsKey("methods" + behaviour.id))»
«protectedRegions.get("methods" + behaviour.id)»
        «ELSE»
            //Add additional options here
        «ENDIF»
    /*PROTECTED REGION END*/

} /* namespace alica */
'''

    def String utilityFunctionCreatorHeader()'''
#pragma once

#include <engine/IUtilityCreator.h>
#include <memory>

namespace alica
{
class IAlicaLogger;

    class UtilityFunctionCreator : public IUtilityCreator
    {
        public:
        virtual ~UtilityFunctionCreator();
        UtilityFunctionCreator();
        std::shared_ptr<BasicUtilityFunction> createUtility(int64_t utilityfunctionConfId);
        void setLogger(IAlicaLogger& logger);
        private:
        IAlicaLogger* _logger;
    };

} /* namespace alica */
'''


    def String utilityFunctionCreatorSource(List<Plan> plans, String pkgName)'''
#include <«pkgName»/UtilityFunctionCreator.h>
«FOR p : plans»
«IF (p.relativeDirectory == null || p.relativeDirectory.isEmpty)»
#include <«pkgName»/«p.name»«p.id».h>
«ELSE»
#include <«pkgName»/«p.relativeDirectory»/«p.name»«p.id».h>
«ENDIF»
«ENDFOR»
#include <iostream>

namespace alica
{

    UtilityFunctionCreator::~UtilityFunctionCreator() {};

    UtilityFunctionCreator::UtilityFunctionCreator() {};

    void UtilityFunctionCreator::setLogger(IAlicaLogger& logger)
    {
        _logger = &logger;
    }


    std::shared_ptr<BasicUtilityFunction> UtilityFunctionCreator::createUtility(int64_t utilityfunctionConfId)
    {
        switch(utilityfunctionConfId)
        {
            «FOR p : plans»
            case «p.id»:
                return std::make_shared<UtilityFunction«p.id»>(*_logger);
                break;
            «ENDFOR»
            default:
            std::cerr << "UtilityFunctionCreator: Unknown utility requested: " << utilityfunctionConfId << std::endl;
            throw new std::exception();
            break;
        }
    }


}
'''

    def String conditionCreatorHeader()'''
#pragma once

#include <engine/IConditionCreator.h>
#include <memory>
#include <iostream>

namespace alica
{
    class BasicCondition;

    class ConditionCreator : public IConditionCreator
    {
        public:
        ConditionCreator();
        virtual ~ConditionCreator();
        std::shared_ptr<BasicCondition> createConditions(int64_t conditionConfId) override;
    };

} /* namespace alica */
'''
    def String conditionCreatorSource(List<Plan> plans, List<Behaviour> behaviours, List<Condition> conditions, String pkgName) '''
#include <«pkgName»/ConditionCreator.h>
«FOR p : plans»
«IF (p.relativeDirectory == null || p.relativeDirectory.isEmpty)»
#include <«pkgName»/«p.name»«p.id».h>
«ELSE»
#include <«pkgName»/«p.relativeDirectory»/«p.name»«p.id».h>
«ENDIF»
«ENDFOR»
«FOR b : behaviours»
«IF (b.relativeDirectory == null || b.relativeDirectory.isEmpty)»
#include <«pkgName»/«b.name»«b.id».h>
«ELSE»
#include <«pkgName»/«b.relativeDirectory»/«b.name»«b.id».h>
«ENDIF»
«ENDFOR»

namespace alica
{

    ConditionCreator::ConditionCreator()
    {
    }
    ConditionCreator::~ConditionCreator()
    {
    }

    std::shared_ptr<BasicCondition> ConditionCreator::createConditions(int64_t conditionConfId)
    {
        switch (conditionConfId)
        {
            «FOR con : conditions»
            case «con.id»:
            «IF (con instanceof PreCondition)»
                    return std::make_shared<PreCondition«con.id»>();
            «ENDIF»
            «IF (con instanceof RuntimeCondition)»
                return std::make_shared<RunTimeCondition«con.id»>();
            «ENDIF»
            «IF (con instanceof PostCondition)»
                return std::make_shared<PostCondition«con.id»>();
            «ENDIF»
                break;
            «ENDFOR»
            default:
            std::cerr << "ConditionCreator: Unknown condition id requested: " << conditionConfId << std::endl;
            throw new std::exception();
            break;
        }
    }
}
'''

    def String constraintCreatorHeader()'''
#pragma once

#include <engine/IConstraintCreator.h>
#include <memory>

namespace alica
{

    class ConstraintCreator : public IConstraintCreator
    {
        public:
        ConstraintCreator();
        virtual ~ConstraintCreator();
        std::shared_ptr<BasicConstraint> createConstraint(int64_t constraintConfId) override;
    };

} /* namespace alica */
'''

    def String constraintCreatorSource(List<Plan> plans, List<Behaviour> behaviours, List<Condition> conditions, String pkgName)'''
#include <«pkgName»/ConstraintCreator.h>

«FOR plan : plans»
«IF (plan.relativeDirectory == null || plan.relativeDirectory.isEmpty)»
#include <«pkgName»/constraints/«plan.name»«plan.id»Constraints.h>
«ELSE»
#include <«pkgName»/«plan.relativeDirectory»/constraints/«plan.name»«plan.id»Constraints.h>
«ENDIF»
«ENDFOR»
«FOR behaviour : behaviours»
«IF (behaviour.relativeDirectory == null || behaviour.relativeDirectory.isEmpty)»
#include <«pkgName»/constraints/«behaviour.name»«behaviour.id»Constraints.h>
«ELSE»
#include <«pkgName»/«behaviour.relativeDirectory»/constraints/«behaviour.name»«behaviour.id»Constraints.h>
«ENDIF»
«ENDFOR»

#include <iostream>

namespace alica
{

    ConstraintCreator::ConstraintCreator()
    {
    }

    ConstraintCreator::~ConstraintCreator()
    {
    }


    std::shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(int64_t constraintConfId)
    {
        switch(constraintConfId)
        {
            «FOR c : conditions»
                «IF (c.variables.size > 0) || (c.quantifiers.size > 0)»
                    case «c.id»:
                    return std::make_shared<Constraint«c.id»>();
                    break;
                «ENDIF»
            «ENDFOR»
            default:
            std::cerr << "ConstraintCreator: Unknown constraint requested: " << constraintConfId << std::endl;
            throw new std::exception();
            break;
        }
    }


}
'''

    def String behaviourConditionHeader(Behaviour behaviour, String pkgName)'''


#include <«pkgName»/DomainCondition.h>
/*PROTECTED REGION ID(incBC«behaviour.id») ENABLED START*/
«IF (protectedRegions.containsKey("incBC" + behaviour.id))»
«protectedRegions.get("incBC" + behaviour.id)»
«ELSE»
//Add additional includes here
«ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    /*PROTECTED REGION ID(meth«behaviour.id») ENABLED START*/
    «IF (protectedRegions.containsKey("meth" + behaviour.id))»
    «protectedRegions.get("meth" + behaviour.id)»
    «ELSE»
    //Add additional includes here
    «ENDIF»
    /*PROTECTED REGION END*/

    «IF (behaviour.preCondition !== null)»
        class PreCondition«behaviour.preCondition.id» : public DomainCondition
        {
            bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
        };
    «ENDIF»
    «IF (behaviour.runtimeCondition !== null)»
        class RunTimeCondition«behaviour.runtimeCondition.id» : public DomainCondition
        {
            bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
        };
    «ENDIF»
    «IF (behaviour.postCondition !== null)»
        class PostCondition«behaviour.postCondition.id» : public DomainCondition
        {
            bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
        };
    «ENDIF»
} /* namespace alica */
'''

    def String behaviourConditionSource(Behaviour behaviour, IConstraintCodeGenerator constraintCodeGenerator, String pkgName) '''
«IF (behaviour.relativeDirectory == null || behaviour.relativeDirectory.isEmpty)»
#include <«pkgName»/«behaviour.name»«behaviour.id».h>
«ELSE»
#include <«pkgName»/«behaviour.relativeDirectory»/«behaviour.name»«behaviour.id».h>
«ENDIF»
#include <memory>

/*PROTECTED REGION ID(inccppBC«behaviour.id») ENABLED START*/
    «IF (protectedRegions.containsKey("inccppBC" + behaviour.id))»
«protectedRegions.get("inccppBC" + behaviour.id)»
    «ELSE»
        //Add additional includes here
    «ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    //Behaviour:«behaviour.name»
    «constraintCodeGenerator.expressionsBehaviourCheckingMethods(behaviour)»

} /* namespace alica */
'''

    def String constraintsHeader(Behaviour behaviour)'''
#pragma once

#include <engine/BasicConstraint.h>
#include <memory>
#include <iostream>

namespace alica
{
    class ProblemDescriptor;
    class RunningPlan;

    «IF (behaviour.preCondition !== null)»
    «IF (behaviour.preCondition.variables.size > 0) || (behaviour.preCondition.quantifiers.size > 0)»
        class Constraint«behaviour.preCondition.id» : public BasicConstraint
        {
            void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
        };
    «ENDIF»
    «ENDIF»
    «IF (behaviour.runtimeCondition !== null)»
    «IF (behaviour.runtimeCondition.variables.size > 0) || (behaviour.runtimeCondition.quantifiers.size > 0)»
        class Constraint«behaviour.runtimeCondition.id» : public BasicConstraint
        {
            void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
        };
    «ENDIF»
    «ENDIF»
    «IF (behaviour.postCondition !== null)»
    «IF (behaviour.postCondition.variables.size > 0) || (behaviour.postCondition.quantifiers.size > 0)»
        class Constraint«behaviour.postCondition.id» : public BasicConstraint
        {
            void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
        };
    «ENDIF»
    «ENDIF»
} /* namespace alica */
'''

    def String constraintsSource(Behaviour behaviour, IConstraintCodeGenerator constraintCodeGenerator, String pkgName) '''
«IF (behaviour.relativeDirectory == null || behaviour.relativeDirectory.isEmpty)»
#include <«pkgName»/constraints/«behaviour.name»«behaviour.id»Constraints.h>
«ELSE»
#include <«pkgName»/«behaviour.relativeDirectory»/constraints/«behaviour.name»«behaviour.id»Constraints.h>
«ENDIF»
/*PROTECTED REGION ID(ch«behaviour.id») ENABLED START*/
        «IF (protectedRegions.containsKey("ch" + behaviour.id))»
«protectedRegions.get("ch" + behaviour.id)»
        «ELSE»
            //Add additional options here
        «ENDIF»
/*PROTECTED REGION END*/

namespace alica
{

    //Behaviour:«behaviour.name»
     «constraintCodeGenerator.constraintBehaviourCheckingMethods(behaviour)»
}
'''

    def String constraintsHeader(Plan plan)'''
#pragma once

#include <engine/BasicConstraint.h>
#include <memory>
#include <iostream>

namespace alica
{
    class ProblemDescriptor;
    class RunningPlan;
    «IF (plan.preCondition !== null)»
    «IF (plan.preCondition.variables.size > 0) || (plan.preCondition.quantifiers.size > 0)»
        class Constraint«plan.preCondition.id» : public BasicConstraint
        {
            void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
        };
    «ENDIF»
    «ENDIF»
    «IF (plan.runtimeCondition !== null)»
    «IF (plan.runtimeCondition.variables.size > 0) || (plan.runtimeCondition.quantifiers.size > 0)»
        class Constraint«plan.runtimeCondition.id» : public BasicConstraint
        {
            void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
        };
    «ENDIF»
    «ENDIF»
    «var  List<Transition> trans = plan.transitions»
    «FOR transition : trans»
    «IF (transition.preCondition !== null)»
    «IF (transition.preCondition.variables.size > 0) || (transition.preCondition.quantifiers.size > 0)»
    class Constraint«transition.preCondition.id» : public BasicConstraint
    {
        void getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp);
    };
    «ENDIF»
    «ENDIF»
    «ENDFOR»
} /* namespace alica */
'''

    def String constraintsSource(Plan plan, IConstraintCodeGenerator constraintCodeGenerator, String pkgName) '''
«IF (plan.relativeDirectory == null || plan.relativeDirectory.isEmpty)»
#include <«pkgName»/constraints/«plan.name»«plan.id»Constraints.h>
«ELSE»
#include <«pkgName»/«plan.relativeDirectory»/constraints/«plan.name»«plan.id»Constraints.h>
«ENDIF»
/*PROTECTED REGION ID(ch«plan.id») ENABLED START*/
        «IF (protectedRegions.containsKey("ch" + plan.id))»
«protectedRegions.get("ch" + plan.id)»
        «ELSE»
            //Add additional options here
        «ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    //Plan:«plan.name»
    /*
    * Tasks: «var List<EntryPoint> entryPoints = plan.entryPoints» «FOR  planEntryPoint : entryPoints»
    * - EntryPoint:«planEntryPoint.id» : «planEntryPoint.task.name» («planEntryPoint.task.id»)«ENDFOR»
    *
    * States:«var List<State> states = plan.states» «FOR state : states»
    * - «state.name» («state.id»)«ENDFOR»
    *
    * Vars:«var List<Variable> variables = plan.variables» «FOR variable : variables»
    * - «variable.name» («variable.id») «ENDFOR»
    */
    «constraintCodeGenerator.constraintPlanCheckingMethods(plan)»
    «FOR state : states»
        «constraintCodeGenerator.constraintStateCheckingMethods(state)»
	«ENDFOR»
}
'''

def String domainPlanHeader() '''
#pragma once

#include <engine/BasicPlan.h>
#include <string>
/*PROTECTED REGION ID(domainPlanHeaderHead) ENABLED START*/
        «IF (protectedRegions.containsKey("domainPlanHeaderHead"))»
«protectedRegions.get("domainPlanHeaderHead")»
        «ELSE»
            //Add additional options here
        «ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    class DomainPlan : public BasicPlan
    {
        public:
        DomainPlan(PlanContext& context);
        virtual ~DomainPlan();
        /*PROTECTED REGION ID(domainPlanClassDecl) ENABLED START*/
«IF (protectedRegions.containsKey("domainPlanClassDecl"))»
«protectedRegions.get("domainPlanClassDecl")»
«ELSE»
        //Add additional options here
«ENDIF»
        /*PROTECTED REGION END*/
    };
} /* namespace alica */
'''

    def String domainBehaviourHeader() '''
#pragma once

#include <engine/BasicBehaviour.h>
#include <string>
/*PROTECTED REGION ID(domainBehaviourHeaderHead) ENABLED START*/
        «IF (protectedRegions.containsKey("domainBehaviourHeaderHead"))»
«protectedRegions.get("domainBehaviourHeaderHead")»
        «ELSE»
            //Add additional options here
        «ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    class DomainBehaviour : public BasicBehaviour
    {
        public:
        DomainBehaviour(BehaviourContext& context);
        virtual ~DomainBehaviour();

        /*PROTECTED REGION ID(domainBehaviourClassDecl) ENABLED START*/
«IF (protectedRegions.containsKey("domainBehaviourClassDecl"))»
«protectedRegions.get("domainBehaviourClassDecl")»
«ELSE»
        //Add additional options here
«ENDIF»
        /*PROTECTED REGION END*/
    };
} /* namespace alica */
'''

def String domainPlanSource(String pkgName) '''
#include <«pkgName»/DomainPlan.h>
/*PROTECTED REGION ID(domainPlanSrcHeaders) ENABLED START*/
«IF (protectedRegions.containsKey("domainPlanSrcHeaders"))»
«protectedRegions.get("domainPlanSrcHeaders")»
«ELSE»
//Add additional options here
«ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    DomainPlan::DomainPlan(PlanContext& context) : BasicPlan(context)
    {
        /*PROTECTED REGION ID(domainPlanConstructor) ENABLED START*/
«IF (protectedRegions.containsKey("domainPlanConstructor"))»
«protectedRegions.get("domainPlanConstructor")»
«ELSE»
        //Add additional options here
«ENDIF»
        /*PROTECTED REGION END*/
    }

    DomainPlan::~DomainPlan()
    {
        /*PROTECTED REGION ID(domainPlanDestructor) ENABLED START*/
«IF (protectedRegions.containsKey("domainPlanDestructor"))»
«protectedRegions.get("domainPlanDestructor")»
«ELSE»
        //Add additional options here
«ENDIF»
        /*PROTECTED REGION END*/
    }

    /*PROTECTED REGION ID(domainPlanMethods) ENABLED START*/
«IF (protectedRegions.containsKey("domainPlanMethods"))»
«protectedRegions.get("domainPlanMethods")»
«ELSE»
    //Add additional options here
«ENDIF»
    /*PROTECTED REGION END*/
} /* namespace alica */
'''

    def String domainBehaviourSource(String pkgName) '''
#include <«pkgName»/DomainBehaviour.h>
/*PROTECTED REGION ID(domainBehaviourSrcHeaders) ENABLED START*/
«IF (protectedRegions.containsKey("domainBehaviourSrcHeaders"))»
«protectedRegions.get("domainBehaviourSrcHeaders")»
«ELSE»
//Add additional options here
«ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    DomainBehaviour::DomainBehaviour(BehaviourContext& context) : BasicBehaviour(context)
    {
        /*PROTECTED REGION ID(domainBehaviourConstructor) ENABLED START*/
«IF (protectedRegions.containsKey("domainBehaviourConstructor"))»
«protectedRegions.get("domainBehaviourConstructor")»
«ELSE»
        //Add additional options here
«ENDIF»
        /*PROTECTED REGION END*/
    }

    DomainBehaviour::~DomainBehaviour()
    {
        /*PROTECTED REGION ID(domainBehaviourDestructor) ENABLED START*/
«IF (protectedRegions.containsKey("domainBehaviourDestructor"))»
«protectedRegions.get("domainBehaviourDestructor")»
«ELSE»
        //Add additional options here
«ENDIF»
        /*PROTECTED REGION END*/
    }

    /*PROTECTED REGION ID(domainBehaviourMethods) ENABLED START*/
«IF (protectedRegions.containsKey("domainBehaviourMethods"))»
«protectedRegions.get("domainBehaviourMethods")»
«ELSE»
    //Add additional options here
«ENDIF»
    /*PROTECTED REGION END*/
} /* namespace alica */
'''

    def String domainConditionHeader() '''
#pragma once

#include <engine/BasicCondition.h>
/*PROTECTED REGION ID(domainHeaderAdditional) ENABLED START*/
«IF (protectedRegions.containsKey("domainHeaderAdditional"))»
«protectedRegions.get("domainHeaderAdditional")»
«ELSE»
        //Add additional options here
«ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    class DomainCondition : public BasicCondition
    {
        public:
        DomainCondition();
        virtual ~DomainCondition();

        /*PROTECTED REGION ID(domainHeader) ENABLED START*/
«IF (protectedRegions.containsKey("domainHeader"))»
«protectedRegions.get("domainHeader")»
«ELSE»
        //Add additional options here
«ENDIF»

        /*PROTECTED REGION END*/

protected:
                /*PROTECTED REGION ID(protectedDomainHeader) ENABLED START*/
«IF (protectedRegions.containsKey("protectedDomainHeader"))»
«protectedRegions.get("protectedDomainHeader")»
«ELSE»
        //Add additional options here
«ENDIF»

        /*PROTECTED REGION END*/

private:
                /*PROTECTED REGION ID(privateDomainHeader) ENABLED START*/
«IF (protectedRegions.containsKey("privateDomainHeader"))»
«protectedRegions.get("privateDomainHeader")»
«ELSE»
        //Add additional options here
«ENDIF»
        /*PROTECTED REGION END*/
    };
} /* namespace alica */
'''

    def String domainConditionSource(String pkgName) '''
#include <«pkgName»/DomainCondition.h>
/*PROTECTED REGION ID(domainSourceHeaders) ENABLED START*/
«IF (protectedRegions.containsKey("domainSourceHeaders"))»
«protectedRegions.get("domainSourceHeaders")»
«ELSE»
        //Add additional options here
«ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    DomainCondition::DomainCondition() : BasicCondition()
    {
        /*PROTECTED REGION ID(domainSourceConstructor) ENABLED START*/
«IF (protectedRegions.containsKey("domainSourceConstructor"))»
«protectedRegions.get("domainSourceConstructor")»
«ELSE»
        //Add additional options here
«ENDIF»
        /*PROTECTED REGION END*/
    }

    DomainCondition::~DomainCondition()
    {
        /*PROTECTED REGION ID(domainSourceDestructor) ENABLED START*/
«IF (protectedRegions.containsKey("domainSourceDestructor"))»
«protectedRegions.get("domainSourceDestructor")»
«ELSE»
        //Add additional options here
«ENDIF»
        /*PROTECTED REGION END*/
    }

    /*PROTECTED REGION ID(additionalMethodsDomainCondition) ENABLED START*/
«IF (protectedRegions.containsKey("additionalMethodsDomainCondition"))»
«protectedRegions.get("additionalMethodsDomainCondition")»
«ELSE»
        //Add additional methods here
«ENDIF»
    /*PROTECTED REGION END*/
} /* namespace alica */
'''

    def String planHeader(Plan plan, String pkgName) '''
#pragma once

#include <«pkgName»/DomainPlan.h>
#include <«pkgName»/DomainCondition.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
/*PROTECTED REGION ID(incl«plan.id») ENABLED START*/
«IF (protectedRegions.containsKey("incl" + plan.id))»
«protectedRegions.get("incl" + plan.id)»
«ELSE»
//Add additional includes here
«ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
    /*PROTECTED REGION ID(meth«plan.id») ENABLED START*/
    «IF (protectedRegions.containsKey("meth" + plan.id))»
«protectedRegions.get("meth" + plan.id)»
    «ELSE»
        //Add additional options here
    «ENDIF»
    /*PROTECTED REGION END*/
    class «plan.name»«plan.id» : public DomainPlan
        {
            public:
                «plan.name»«plan.id»(PlanContext& context);
                virtual ~«plan.name»«plan.id»();
                /*PROTECTED REGION ID(pub«plan.id») ENABLED START*/
                «IF (protectedRegions.containsKey("pub" + plan.id))»
    «protectedRegions.get("pub" + plan.id)»
                «ELSE»
                //Add additional public methods here
                «ENDIF»
                /*PROTECTED REGION END*/
            protected:
                /*PROTECTED REGION ID(pro«plan.id») ENABLED START*/
                «IF (protectedRegions.containsKey("pro" + plan.id))»
    «protectedRegions.get("pro" + plan.id)»
                «ELSE»
                // Override these methods for your use case
                // virtual void run(void* msg) override;
                // virtual void onInit() override;
                // virtual void onTerminate() override;
                // Add additional protected methods here
                «ENDIF»
                /*PROTECTED REGION END*/
            private:
            /*PROTECTED REGION ID(prv«plan.id») ENABLED START*/
                «IF (protectedRegions.containsKey("prv" + plan.id))»
    «protectedRegions.get("prv" + plan.id)»
                «ELSE»
                //Add additional private methods here
                «ENDIF»
            /*PROTECTED REGION END*/
        };

    «var  List<State> states =  plan.states»

    class UtilityFunction«plan.id» : public BasicUtilityFunction
    {
    public:
        UtilityFunction«plan.id»(IAlicaLogger& logger);
        std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
    };
    «IF (plan.preCondition !== null)»
        class PreCondition«plan.preCondition.id» : public DomainCondition
        {
            bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
        };
    «ENDIF»
    «IF (plan.runtimeCondition !== null)»
        class RunTimeCondition«plan.runtimeCondition.id» : public DomainCondition
        {
            bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
        };
    «ENDIF»
    «FOR s : states»
        «IF s instanceof TerminalState»
            «var TerminalState terminalSate = s as TerminalState»
            «IF terminalSate.postCondition !== null»
                class PostCondition«terminalSate.postCondition.id» : public DomainCondition
                {
                    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
                };
            «ENDIF»
        «ENDIF»
        «var List<Transition> transitions = s.outTransitions»
        «FOR transition : transitions»
            «IF transition.preCondition !== null»
                class PreCondition«transition.preCondition.id» : public DomainCondition
                {
                    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
                };
            «ENDIF»
        «ENDFOR»
    «ENDFOR»
} /* namespace alica */
'''

    def String planSource(Plan plan, IConstraintCodeGenerator constraintCodeGenerator, String pkgName) '''
«IF (plan.relativeDirectory == null || plan.relativeDirectory.isEmpty)»
#include <«pkgName»/«plan.name»«plan.id».h>
«ELSE»
#include <«pkgName»/«plan.relativeDirectory»/«plan.name»«plan.id».h>
«ENDIF»
/*PROTECTED REGION ID(eph«plan.id») ENABLED START*/
«IF (protectedRegions.containsKey("eph" + plan.id))»
«protectedRegions.get("eph" + plan.id)»
«ELSE»
    //Add additional options here
«ENDIF»
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  «plan.name» («plan.id»)
//
// Tasks: «var  List<EntryPoint> entryPoints = plan.entryPoints»
«FOR planEntryPoint : entryPoints»
//   - «planEntryPoint.task.name» («planEntryPoint.task.id») (Entrypoint: «planEntryPoint.id»)«ENDFOR»
//
// States: «var  List<State> states = plan.states»
«FOR stateOfInPlan : states»
//   - «stateOfInPlan.name» («stateOfInPlan.id»)
«ENDFOR»
    «plan.name»«plan.id»::«plan.name»«plan.id»(PlanContext& context) : DomainPlan(context)
    {
    /*PROTECTED REGION ID(con«plan.id») ENABLED START*/
    «IF (protectedRegions.containsKey("con" + plan.id))»
«protectedRegions.get("con" + plan.id)»
    «ELSE»
        //Add additional options here
    «ENDIF»
    /*PROTECTED REGION END*/
    }
    «plan.name»«plan.id»::~«plan.name»«plan.id»()
    {
        /*PROTECTED REGION ID(dcon«plan.id») ENABLED START*/
        «IF (protectedRegions.containsKey("dcon" + plan.id))»
«protectedRegions.get("dcon" + plan.id)»
        «ELSE»
            //Add additional options here
        «ENDIF»
        /*PROTECTED REGION END*/

    }

«constraintCodeGenerator.expressionsPlanCheckingMethods(plan)»
/**
«FOR entryPoint : entryPoints»
* Task: «entryPoint.task.name»  -> EntryPoint-ID: «entryPoint.id»
«ENDFOR»
*/

UtilityFunction«plan.id»::UtilityFunction«plan.id»(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{}

std::shared_ptr<UtilityFunction> UtilityFunction«plan.id»::getUtilityFunction(Plan* plan)
{
   /*PROTECTED REGION ID(«plan.id») ENABLED START*/
   «IF (protectedRegions.containsKey(String.valueOf(plan.id)))»
«protectedRegions.get(String.valueOf(plan.id))»
   «ELSE»
        std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan, _logger);
        return defaultFunction;
   «ENDIF»
    /*PROTECTED REGION END*/
}

«FOR state : states»
    «constraintCodeGenerator.expressionsStateCheckingMethods(state)»

«ENDFOR»

/*PROTECTED REGION ID(methods«plan.id») ENABLED START*/
        «IF (protectedRegions.containsKey("methods" + plan.id))»
«protectedRegions.get("methods" + plan.id)»
        «ELSE»
            //Add additional options here
        «ENDIF»
    /*PROTECTED REGION END*/
}
'''
}
