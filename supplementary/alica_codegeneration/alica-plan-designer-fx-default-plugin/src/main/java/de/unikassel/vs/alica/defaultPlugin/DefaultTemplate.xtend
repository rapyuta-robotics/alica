package de.unikassel.vs.alica.defaultPlugin;

import de.unikassel.vs.alica.planDesigner.alicamodel.Plan
import de.unikassel.vs.alica.planDesigner.alicamodel.Behaviour
import de.unikassel.vs.alica.planDesigner.alicamodel.State
import de.unikassel.vs.alica.planDesigner.alicamodel.ConfAbstractPlanWrapper
import de.unikassel.vs.alica.planDesigner.alicamodel.TerminalState
import de.unikassel.vs.alica.planDesigner.alicamodel.Transition
import de.unikassel.vs.alica.planDesigner.alicamodel.Variable
import de.unikassel.vs.alica.planDesigner.alicamodel.Quantifier
import java.util.Map
import java.util.List
import de.unikassel.vs.alica.planDesigner.alicamodel.EntryPoint
import de.unikassel.vs.alica.planDesigner.alicamodel.AbstractPlan

class DefaultTemplate {

    private Map<String, String> protectedRegions;

    public def void setProtectedRegions (Map<String, String> regions) {
        protectedRegions = regions;
    }

    def String expressionsPlanCheckingMethods(Plan plan) '''
        «IF (plan.preCondition !== null && plan.preCondition.pluginName == "DefaultPlugin")»
            //Check of PreCondition - (Name): «plan.preCondition.name», (ConditionString): «plan.preCondition.conditionString» , (Comment) : «plan.preCondition.comment»

            /**
             * Available Vars:«var  List<Variable> variables =  plan.preCondition.variables»«FOR variable :variables»
             *	- «variable.name» («variable.id»)«ENDFOR»
             */
            bool PreCondition«plan.preCondition.id»::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
            {
                /*PROTECTED REGION ID(«plan.preCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey(plan.preCondition.id + ""))»
                    «protectedRegions.get(plan.preCondition.id + "")»
                «ELSE»
                    std::cout << "The PreCondition «plan.preCondition.id» in Plan '«plan.getName»' is not implement yet!" << std::endl;
                    return false;
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»
        «IF (plan.runtimeCondition !== null && plan.runtimeCondition.pluginName == "DefaultPlugin")»
            //Check of RuntimeCondition - (Name): «plan.runtimeCondition.name», (ConditionString): «plan.runtimeCondition.conditionString», (Comment) : «plan.runtimeCondition.comment»

            /**
             * Available Vars:«var  List<Variable> variables = plan.runtimeCondition.variables»«FOR variable : variables»
             *	- «variable.name» («variable.id»)«ENDFOR»
             */
            bool RunTimeCondition«plan.runtimeCondition.id»::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm) {
                /*PROTECTED REGION ID(«plan.runtimeCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey(plan.runtimeCondition.id + ""))»
                    «protectedRegions.get(plan.runtimeCondition.id + "")»
                «ELSE»
                    std::cout << "The RunTimeCondition «plan.runtimeCondition.id» in Plan '«plan.getName»' is not implement yet!" << std::endl;
                    return false;
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»
        «var  List<State> states =  plan.states»
        «FOR state : states»
        «IF (state instanceof TerminalState)»
        «var TerminalState terminalState = state as TerminalState»
        «IF (terminalState.postCondition !== null && terminalState.postCondition.pluginName == "DefaultPlugin")»
            //Check of PostCondition - (Name): «terminalState.postCondition.name», (ConditionString): «terminalState.postCondition.conditionString» , (Comment) : «terminalState.postCondition.comment»

            /**
             * Available Vars:«var  List<Variable> variables =  terminalState.postCondition.variables»«FOR variable :variables»
             *	- «variable.name» («variable.id»)«ENDFOR»
             */
            bool PostCondition«terminalState.postCondition.id»::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
            {
                /*PROTECTED REGION ID(«terminalState.postCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey(terminalState.postCondition.id + ""))»
                    «protectedRegions.get(terminalState.postCondition.id + "")»
                «ELSE»
                    std::cout << "The PostCondition «terminalState.postCondition.id» in TerminalState '«terminalState.getName»' is not implement yet!" << std::endl;
                    std::cout << "However, PostConditions are a feature that makes sense in the context of planning, which is not supported by ALICA, yet! So don't worry." << std::endl;
                    return false;
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»
        «ENDIF»
        «ENDFOR»
    '''

    def String expressionsBehaviourCheckingMethods(Behaviour behaviour) '''
        «IF (behaviour.runtimeCondition !== null && behaviour.runtimeCondition.pluginName == "DefaultPlugin")»
            //Check of RuntimeCondition - (Name): «behaviour.runtimeCondition.name», (ConditionString): «behaviour.runtimeCondition.conditionString», (Comment) : «behaviour.runtimeCondition.comment»

            /**
             * Available Vars:«var  List<Variable> variables = behaviour.runtimeCondition.variables»«FOR variable : variables»
             *	- «variable.name» («variable.id»)«ENDFOR»
             */
            bool RunTimeCondition«behaviour.runtimeCondition.id»::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm) {
                /*PROTECTED REGION ID(«behaviour.runtimeCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey(behaviour.runtimeCondition.id + ""))»
                    «protectedRegions.get(behaviour.runtimeCondition.id + "")»
                «ELSE»
                    std::cout << "The RuntimeCondition «behaviour.runtimeCondition.id» in Behaviour «behaviour.getName» is not implement yet!" << std::endl;
                    return false;
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»
        «IF (behaviour.preCondition !== null && behaviour.preCondition.pluginName == "DefaultPlugin")»
            //Check of PreCondition - (Name): «behaviour.preCondition.name», (ConditionString): «behaviour.preCondition.conditionString» , (Comment) : «behaviour.preCondition.comment»

            /**
             * Available Vars:«var  List<Variable> variables =  behaviour.preCondition.variables»«FOR variable :variables»
             *	- «variable.name» («variable.id»)«ENDFOR»
             */
            bool PreCondition«behaviour.preCondition.id»::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
            {
                /*PROTECTED REGION ID(«behaviour.preCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey(behaviour.preCondition.id + ""))»
                    «protectedRegions.get(behaviour.preCondition.id + "")»
                «ELSE»
                    std::cout << "The PreCondition «behaviour.preCondition.id» in Behaviour «behaviour.getName» is not implement yet!" << std::endl;
                    return false;
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»
        «IF (behaviour.postCondition !== null && behaviour.postCondition.pluginName == "DefaultPlugin")»
            //Check of PostCondition - (Name): «behaviour.postCondition.name», (ConditionString): «behaviour.postCondition.conditionString» , (Comment) : «behaviour.postCondition.comment»

            /**
             * Available Vars:«var  List<Variable> variables =  behaviour.postCondition.variables»«FOR variable :variables»
             *	- «variable.name» («variable.id»)«ENDFOR»
             */
            bool PostCondition«behaviour.postCondition.id»::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
            {
                /*PROTECTED REGION ID(«behaviour.postCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey(behaviour.postCondition.id + ""))»
                    «protectedRegions.get(behaviour.postCondition.id + "")»
                «ELSE»
                    std::cout << "The PostCondition «behaviour.postCondition.id» in Behaviour '«behaviour.getName»' is not implement, yet!" << std::endl;
                    return false;
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»
    '''

    def String constraintPlanCheckingMethods(Plan plan) '''
        «IF (plan.runtimeCondition !== null && plan.runtimeCondition.pluginName == "DefaultPlugin")»
        «IF (plan.runtimeCondition.variables.size > 0) || (plan.runtimeCondition.quantifiers.size > 0)»
            /**
             * RuntimeCondition - (Name): «plan.runtimeCondition.name»
             * (ConditionString): «plan.runtimeCondition.conditionString»
            «var  List<Variable> variables =  plan.runtimeCondition.variables»
            «IF (variables !== null)»
                 * Static Variables: «FOR variable : variables»«variable.name» «ENDFOR»
            «ENDIF»
             * Domain Variables:
            «var  List<Quantifier> quantifiers =  plan.runtimeCondition.quantifiers»
            «IF (quantifiers !== null)»
                «FOR q : quantifiers»
                    «var  List<String> sorts=  q.sorts»
                    * forall agents in «q.scope.name» let v = «sorts»
                «ENDFOR»
            «ENDIF»
            *
            */
            void Constraint«plan.runtimeCondition.id»::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) {
                /*PROTECTED REGION ID(cc«plan.runtimeCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey("cc" + plan.runtimeCondition.id))»
                    «protectedRegions.get("cc" + plan.runtimeCondition.id)»
                «ELSE»
                    //Please describe your runtime constraint here
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»

        «ENDIF»
        «IF (plan.preCondition !== null && plan.preCondition.pluginName == "DefaultPlugin")»
        «IF (plan.preCondition.variables.size > 0) || (plan.preCondition.quantifiers.size > 0)»
            /**
             * PreCondition - (Name): «plan.preCondition.name»
             * (ConditionString): «plan.preCondition.conditionString»
            «var  List<Variable> variables =  plan.preCondition.variables»
             * Static Variables: «FOR variable : variables»«variable.name» «ENDFOR»
             * Domain Variables:
            «var  List<Quantifier> quantifiers =  plan.preCondition.quantifiers»
            «IF (quantifiers !== null)»
                «FOR q : quantifiers»
                    «var  List<String> sorts=  q.sorts»
                    * forall agents in «q.scope.name» let v = «sorts»
                «ENDFOR»
            «ENDIF»
             *
             */
            void Constraint«plan.preCondition.id»::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) {
                /*PROTECTED REGION ID(cc«plan.preCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey("cc" + plan.preCondition.id))»
                    «protectedRegions.get("cc" + plan.preCondition.id)»
                «ELSE»
                    //Please describe your precondition constraint here
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»
        «ENDIF»
    '''

    def String constraintBehaviourCheckingMethods(Behaviour behaviour) '''
        «IF (behaviour.runtimeCondition !== null && behaviour.runtimeCondition.pluginName == "DefaultPlugin")»
        «IF (behaviour.runtimeCondition.variables.size > 0) || (behaviour.runtimeCondition.quantifiers.size > 0)»
            /**
             * RuntimeCondition - (Name): «behaviour.runtimeCondition.name»
             * (ConditionString): «behaviour.runtimeCondition.conditionString»
            «var  List<Variable> variables =  behaviour.runtimeCondition.variables»
            «IF (variables !== null)»
                 * Static Variables: «FOR variable : variables»«variable.name» «ENDFOR»
            «ENDIF»
             * Domain Variables:
            «var  List<Quantifier> quantifiers =  behaviour.runtimeCondition.quantifiers»
            «IF (quantifiers !== null)»
                «FOR q : quantifiers»
                    «var  List<String> sorts=  q.sorts»
                    * forall agents in «q.scope.name» let v = «sorts»
                «ENDFOR»
            «ENDIF»
            *
            */
            void Constraint«behaviour.runtimeCondition.id»::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) {
                /*PROTECTED REGION ID(cc«behaviour.runtimeCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey("cc" + behaviour.runtimeCondition.id))»
                    «protectedRegions.get("cc" + behaviour.runtimeCondition.id)»
                «ELSE»
                    //Please describe your runtime constraint here
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»
        «ENDIF»
        «IF (behaviour.preCondition !== null && behaviour.preCondition.pluginName == "DefaultPlugin")»
        «IF (behaviour.preCondition.variables.size > 0) || (behaviour.preCondition.quantifiers.size > 0)»
            /**
             * PreCondition - (Name): «behaviour.preCondition.name»
             * (ConditionString): «behaviour.preCondition.conditionString»
            «var  List<Variable> variables =  behaviour.preCondition.variables»
             * Static Variables: «FOR variable : variables»«variable.name» «ENDFOR»
             * Domain Variables:
            «var  List<Quantifier> quantifiers =  behaviour.preCondition.quantifiers»
            «IF (quantifiers !== null)»
                «FOR q : quantifiers»
                    «var  List<String> sorts=  q.sorts»
                    * forall agents in «q.scope.name» let v = «sorts»
                «ENDFOR»
            «ENDIF»
             *
             */
            void Constraint«behaviour.preCondition.id»::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) {
                /*PROTECTED REGION ID(cc«behaviour.preCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey("cc" + behaviour.preCondition.id))»
                    «protectedRegions.get("cc" + behaviour.preCondition.id)»
                «ELSE»
                    //Please describe your precondition constraint here
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»
        «ENDIF»
        «IF (behaviour.postCondition !== null && behaviour.postCondition.pluginName == "DefaultPlugin")»
         «IF (behaviour.postCondition.variables.size > 0) || (behaviour.postCondition.quantifiers.size > 0)»
            /**
             * PostCondition - (Name): «behaviour.postCondition.name»
             * (ConditionString): «behaviour.postCondition.conditionString»
            «var  List<Variable> variables =  behaviour.postCondition.variables»
            «IF (variables !== null)»
                 * Static Variables: «FOR variable : variables»«variable.name» «ENDFOR»
            «ENDIF»
             * Domain Variables:
            «var  List<Quantifier> quantifiers =  behaviour.postCondition.quantifiers»
            «IF (quantifiers !== null)»
                «FOR q : quantifiers»
                    «var  List<String> sorts=  q.sorts»
                    * forall agents in «q.scope.name» let v = «sorts»
                «ENDFOR»
            «ENDIF»
            *
            */
            void Constraint«behaviour.postCondition.id»::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) {
                /*PROTECTED REGION ID(cc«behaviour.postCondition.id») ENABLED START*/
                «IF (protectedRegions.containsKey("cc" + behaviour.postCondition.id))»
                    «protectedRegions.get("cc" + behaviour.postCondition.id)»
                «ELSE»
                    //Please describe your runtime constraint here
                «ENDIF»
                /*PROTECTED REGION END*/
            }
        «ENDIF»
        «ENDIF»
    '''

    def String constraintStateCheckingMethods(State state) '''
        // State: «state.name»
        «var  List<Transition> outTransitions = state.outTransitions»
        «FOR transition : outTransitions»
            «IF transition.preCondition !== null && transition.preCondition.pluginName == "DefaultPlugin"»
                «IF (transition.preCondition.variables.size > 0) || (transition.preCondition.quantifiers.size > 0)»
                    /**
                    * Transition:
                    * - Name: «transition.preCondition.name»
                    * - Comment: «transition.preCondition.comment»
                    * - ConditionString: «transition.preCondition.conditionString»
                    *
                    * «var  List<ConfAbstractPlanWrapper> wrappers = state.confAbstractPlanWrappers»
                    * AbstractPlans in State: «FOR wrapper : wrappers»
                    * - AbstractPlan Name: «wrapper.abstractPlan.name», PlanID: «wrapper.abstractPlan.id» «ENDFOR»
                    «var  List<Variable> variables =  transition.preCondition.variables»
                    «IF (variables !== null)»
                         * Static Variables: «FOR variable : variables»«variable.name» «ENDFOR»
                    «ENDIF»
                    * Domain Variables:
                    «IF transition.preCondition.quantifiers !== null && transition.preCondition.quantifiers.size > 0»
                        «var  List<Quantifier> quantifiers = transition.preCondition.quantifiers»
                        «FOR q : quantifiers»«var  List<String> sorts=  q.sorts»
                    * forall agents in «q.scope.name» let v = «sorts»
                        «ENDFOR»
                    «ENDIF»
                     */
                    void Constraint«transition.preCondition.id»::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) {
                        /*PROTECTED REGION ID(cc«transition.preCondition.id») ENABLED START*/
                        «IF (protectedRegions.containsKey("cc" + transition.preCondition.id))»
                            «protectedRegions.get("cc" + transition.preCondition.id)»
                        «ELSE»
                            //Please describe your precondition constraint here
                        «ENDIF»
                        /*PROTECTED REGION END*/
                    }
                «ENDIF»
            «ENDIF»
        «ENDFOR»
    '''
}