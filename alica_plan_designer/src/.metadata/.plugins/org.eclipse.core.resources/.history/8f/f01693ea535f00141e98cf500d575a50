/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica.util;

import de.uni_kassel.vs.cn.planDesigner.alica.*;
import java.util.Map;
import org.eclipse.emf.common.notify.Adapter;
import org.eclipse.emf.common.notify.Notifier;
import org.eclipse.emf.common.notify.impl.AdapterFactoryImpl;
import org.eclipse.emf.ecore.EObject;
import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.CapValue;
import de.uni_kassel.vs.cn.planDesigner.alica.Capability;
import de.uni_kassel.vs.cn.planDesigner.alica.CapabilityDefinitionSet;
import de.uni_kassel.vs.cn.planDesigner.alica.Characteristic;
import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Edge;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.FailureState;
import de.uni_kassel.vs.cn.planDesigner.alica.ForallAgents;
import de.uni_kassel.vs.cn.planDesigner.alica.IInhabitable;
import de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.alica.Node;
import de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;
import de.uni_kassel.vs.cn.planDesigner.alica.PostCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.PreCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.Quantifier;
import de.uni_kassel.vs.cn.planDesigner.alica.Rating;
import de.uni_kassel.vs.cn.planDesigner.alica.Role;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleDefinitionSet;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleSet;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.alica.RuntimeCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.SuccessState;
import de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskRepository;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskWrapper;
import de.uni_kassel.vs.cn.planDesigner.alica.TerminalState;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.alica.Variable;

/**
 * <!-- begin-user-doc -->
 * The <b>Adapter Factory</b> for the model.
 * It provides an adapter <code>createXXX</code> method for each class of the model.
 * <!-- end-user-doc -->
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage
 * @generated
 */
public class AlicaAdapterFactory extends AdapterFactoryImpl {
	/**
	 * The cached model package.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected static AlicaPackage modelPackage;

	/**
	 * Creates an instance of the adapter factory.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public AlicaAdapterFactory() {
		if (modelPackage == null) {
			modelPackage = AlicaPackage.eINSTANCE;
		}
	}

	/**
	 * Returns whether this factory is applicable for the type of the object.
	 * <!-- begin-user-doc -->
	 * This implementation returns <code>true</code> if the object is either the model's package or is an instance object of the model.
	 * <!-- end-user-doc -->
	 * @return whether this factory is applicable for the type of the object.
	 * @generated
	 */
	@Override
	public boolean isFactoryForType(Object object) {
		if (object == modelPackage) {
			return true;
		}
		if (object instanceof EObject) {
			return ((EObject)object).eClass().getEPackage() == modelPackage;
		}
		return false;
	}

	/**
	 * The switch that delegates to the <code>createXXX</code> methods.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected AlicaSwitch<Adapter> modelSwitch =
		new AlicaSwitch<Adapter>() {
			@Override
			public Adapter caseTransition(Transition object) {
				return createTransitionAdapter();
			}
			@Override
			public Adapter caseCondition(Condition object) {
				return createConditionAdapter();
			}
			@Override
			public Adapter casePreCondition(PreCondition object) {
				return createPreConditionAdapter();
			}
			@Override
			public Adapter caseEntryPoint(EntryPoint object) {
				return createEntryPointAdapter();
			}
			@Override
			public Adapter caseTerminalState(TerminalState object) {
				return createTerminalStateAdapter();
			}
			@Override
			public Adapter caseSuccessState(SuccessState object) {
				return createSuccessStateAdapter();
			}
			@Override
			public Adapter caseFailureState(FailureState object) {
				return createFailureStateAdapter();
			}
			@Override
			public Adapter caseAbstractPlan(AbstractPlan object) {
				return createAbstractPlanAdapter();
			}
			@Override
			public Adapter caseBehaviour(Behaviour object) {
				return createBehaviourAdapter();
			}
			@Override
			public Adapter caseState(State object) {
				return createStateAdapter();
			}
			@Override
			public Adapter casePlan(Plan object) {
				return createPlanAdapter();
			}
			@Override
			public Adapter casePlanType(PlanType object) {
				return createPlanTypeAdapter();
			}
			@Override
			public Adapter caseRating(Rating object) {
				return createRatingAdapter();
			}
			@Override
			public Adapter casePostCondition(PostCondition object) {
				return createPostConditionAdapter();
			}
			@Override
			public Adapter caseRuntimeCondition(RuntimeCondition object) {
				return createRuntimeConditionAdapter();
			}
			@Override
			public Adapter casePlanElement(PlanElement object) {
				return createPlanElementAdapter();
			}
			@Override
			public Adapter caseTask(Task object) {
				return createTaskAdapter();
			}
			@Override
			public Adapter caseEStringToEStringMapEntry(Map.Entry<String, String> object) {
				return createEStringToEStringMapEntryAdapter();
			}
			@Override
			public Adapter caseBehaviourConfiguration(BehaviourConfiguration object) {
				return createBehaviourConfigurationAdapter();
			}
			@Override
			public Adapter caseRole(Role object) {
				return createRoleAdapter();
			}
			@Override
			public Adapter caseRoleSet(RoleSet object) {
				return createRoleSetAdapter();
			}
			@Override
			public Adapter caseELongToDoubleMapEntry(Map.Entry<Long, Double> object) {
				return createELongToDoubleMapEntryAdapter();
			}
			@Override
			public Adapter caseRoleDefinitionSet(RoleDefinitionSet object) {
				return createRoleDefinitionSetAdapter();
			}
			@Override
			public Adapter caseRoleTaskMapping(RoleTaskMapping object) {
				return createRoleTaskMappingAdapter();
			}
			@Override
			public Adapter caseCharacteristic(Characteristic object) {
				return createCharacteristicAdapter();
			}
			@Override
			public Adapter caseTaskGraph(TaskGraph object) {
				return createTaskGraphAdapter();
			}
			@Override
			public Adapter caseEdge(Edge object) {
				return createEdgeAdapter();
			}
			@Override
			public Adapter caseTaskWrapper(TaskWrapper object) {
				return createTaskWrapperAdapter();
			}
			@Override
			public Adapter caseInternalRoleTaskMapping(InternalRoleTaskMapping object) {
				return createInternalRoleTaskMappingAdapter();
			}
			@Override
			public Adapter caseNode(Node object) {
				return createNodeAdapter();
			}
			@Override
			public Adapter caseTaskRepository(TaskRepository object) {
				return createTaskRepositoryAdapter();
			}
			@Override
			public Adapter caseSynchronisation(Synchronisation object) {
				return createSynchronisationAdapter();
			}
			@Override
			public Adapter caseVariable(Variable object) {
				return createVariableAdapter();
			}
			@Override
			public Adapter caseParametrisation(Parametrisation object) {
				return createParametrisationAdapter();
			}
			@Override
			public Adapter caseAnnotatedPlan(AnnotatedPlan object) {
				return createAnnotatedPlanAdapter();
			}
			@Override
			public Adapter caseQuantifier(Quantifier object) {
				return createQuantifierAdapter();
			}
			@Override
			public Adapter caseForallAgents(ForallAgents object) {
				return createForallAgentsAdapter();
			}
			@Override
			public Adapter caseIInhabitable(IInhabitable object) {
				return createIInhabitableAdapter();
			}
			@Override
			public Adapter caseCapability(Capability object) {
				return createCapabilityAdapter();
			}
			@Override
			public Adapter caseCapValue(CapValue object) {
				return createCapValueAdapter();
			}
			@Override
			public Adapter caseCapabilityDefinitionSet(CapabilityDefinitionSet object) {
				return createCapabilityDefinitionSetAdapter();
			}
			@Override
			public Adapter casePlanningProblem(PlanningProblem object) {
				return createPlanningProblemAdapter();
			}
			@Override
			public Adapter casePlanner(Planner object) {
				return createPlannerAdapter();
			}
			@Override
			public Adapter caseFluent(Fluent object) {
				return createFluentAdapter();
			}
			@Override
			public Adapter caseDomainDescription(DomainDescription object) {
				return createDomainDescriptionAdapter();
			}
			@Override
			public Adapter casePlanners(Planners object) {
				return createPlannersAdapter();
			}
			@Override
			public Adapter caseEStringToEObjectMapEntry(Map.Entry<String, Object> object) {
				return createEStringToEObjectMapEntryAdapter();
			}
			@Override
			public Adapter caseFluentParameters(FluentParameters object) {
				return createFluentParametersAdapter();
			}
			@Override
			public Adapter caseConstant(Constant object) {
				return createConstantAdapter();
			}
			@Override
			public Adapter caseBehaviourCreator(BehaviourCreator object) {
				return createBehaviourCreatorAdapter();
			}
			@Override
			public Adapter caseConditionCreator(ConditionCreator object) {
				return createConditionCreatorAdapter();
			}
			@Override
			public Adapter caseUtilityFunctionCreator(UtilityFunctionCreator object) {
				return createUtilityFunctionCreatorAdapter();
			}
			@Override
			public Adapter caseConstraintCreator(ConstraintCreator object) {
				return createConstraintCreatorAdapter();
			}
			@Override
			public Adapter defaultCase(EObject object) {
				return createEObjectAdapter();
			}
		};

	/**
	 * Creates an adapter for the <code>target</code>.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param target the object to adapt.
	 * @return the adapter for the <code>target</code>.
	 * @generated
	 */
	@Override
	public Adapter createAdapter(Notifier target) {
		return modelSwitch.doSwitch((EObject)target);
	}


	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Transition <em>Transition</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Transition
	 * @generated
	 */
	public Adapter createTransitionAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Condition <em>Condition</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Condition
	 * @generated
	 */
	public Adapter createConditionAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.PreCondition <em>Pre Condition</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.PreCondition
	 * @generated
	 */
	public Adapter createPreConditionAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint <em>Entry Point</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint
	 * @generated
	 */
	public Adapter createEntryPointAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.TerminalState <em>Terminal State</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.TerminalState
	 * @generated
	 */
	public Adapter createTerminalStateAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.SuccessState <em>Success State</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.SuccessState
	 * @generated
	 */
	public Adapter createSuccessStateAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.FailureState <em>Failure State</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.FailureState
	 * @generated
	 */
	public Adapter createFailureStateAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan <em>Abstract Plan</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan
	 * @generated
	 */
	public Adapter createAbstractPlanAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Behaviour <em>Behaviour</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Behaviour
	 * @generated
	 */
	public Adapter createBehaviourAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.State <em>State</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.State
	 * @generated
	 */
	public Adapter createStateAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Plan <em>Plan</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Plan
	 * @generated
	 */
	public Adapter createPlanAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanType <em>Plan Type</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.PlanType
	 * @generated
	 */
	public Adapter createPlanTypeAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Rating <em>Rating</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Rating
	 * @generated
	 */
	public Adapter createRatingAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.PostCondition <em>Post Condition</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.PostCondition
	 * @generated
	 */
	public Adapter createPostConditionAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.RuntimeCondition <em>Runtime Condition</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.RuntimeCondition
	 * @generated
	 */
	public Adapter createRuntimeConditionAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanElement <em>Plan Element</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.PlanElement
	 * @generated
	 */
	public Adapter createPlanElementAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Task <em>Task</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Task
	 * @generated
	 */
	public Adapter createTaskAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link java.util.Map.Entry <em>EString To EString Map Entry</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see java.util.Map.Entry
	 * @generated
	 */
	public Adapter createEStringToEStringMapEntryAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration <em>Behaviour Configuration</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration
	 * @generated
	 */
	public Adapter createBehaviourConfigurationAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Role <em>Role</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Role
	 * @generated
	 */
	public Adapter createRoleAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.RoleSet <em>Role Set</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.RoleSet
	 * @generated
	 */
	public Adapter createRoleSetAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link java.util.Map.Entry <em>ELong To Double Map Entry</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see java.util.Map.Entry
	 * @generated
	 */
	public Adapter createELongToDoubleMapEntryAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.RoleDefinitionSet <em>Role Definition Set</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.RoleDefinitionSet
	 * @generated
	 */
	public Adapter createRoleDefinitionSetAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping <em>Role Task Mapping</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping
	 * @generated
	 */
	public Adapter createRoleTaskMappingAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Characteristic <em>Characteristic</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Characteristic
	 * @generated
	 */
	public Adapter createCharacteristicAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph <em>Task Graph</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph
	 * @generated
	 */
	public Adapter createTaskGraphAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Edge <em>Edge</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Edge
	 * @generated
	 */
	public Adapter createEdgeAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.TaskWrapper <em>Task Wrapper</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.TaskWrapper
	 * @generated
	 */
	public Adapter createTaskWrapperAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping <em>Internal Role Task Mapping</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping
	 * @generated
	 */
	public Adapter createInternalRoleTaskMappingAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Node <em>Node</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Node
	 * @generated
	 */
	public Adapter createNodeAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.TaskRepository <em>Task Repository</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.TaskRepository
	 * @generated
	 */
	public Adapter createTaskRepositoryAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation <em>Synchronisation</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation
	 * @generated
	 */
	public Adapter createSynchronisationAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Variable <em>Variable</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Variable
	 * @generated
	 */
	public Adapter createVariableAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation <em>Parametrisation</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation
	 * @generated
	 */
	public Adapter createParametrisationAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan <em>Annotated Plan</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan
	 * @generated
	 */
	public Adapter createAnnotatedPlanAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Quantifier <em>Quantifier</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Quantifier
	 * @generated
	 */
	public Adapter createQuantifierAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.ForallAgents <em>Forall Agents</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.ForallAgents
	 * @generated
	 */
	public Adapter createForallAgentsAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.IInhabitable <em>IInhabitable</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.IInhabitable
	 * @generated
	 */
	public Adapter createIInhabitableAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Capability <em>Capability</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Capability
	 * @generated
	 */
	public Adapter createCapabilityAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.CapValue <em>Cap Value</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.CapValue
	 * @generated
	 */
	public Adapter createCapValueAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.CapabilityDefinitionSet <em>Capability Definition Set</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.CapabilityDefinitionSet
	 * @generated
	 */
	public Adapter createCapabilityDefinitionSetAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem <em>Planning Problem</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem
	 * @generated
	 */
	public Adapter createPlanningProblemAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Planner <em>Planner</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Planner
	 * @generated
	 */
	public Adapter createPlannerAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Fluent <em>Fluent</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Fluent
	 * @generated
	 */
	public Adapter createFluentAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.DomainDescription <em>Domain Description</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.DomainDescription
	 * @generated
	 */
	public Adapter createDomainDescriptionAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Planners <em>Planners</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Planners
	 * @generated
	 */
	public Adapter createPlannersAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link java.util.Map.Entry <em>EString To EObject Map Entry</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see java.util.Map.Entry
	 * @generated
	 */
	public Adapter createEStringToEObjectMapEntryAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.FluentParameters <em>Fluent Parameters</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.FluentParameters
	 * @generated
	 */
	public Adapter createFluentParametersAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.Constant <em>Constant</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Constant
	 * @generated
	 */
	public Adapter createConstantAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.BehaviourCreator <em>Behaviour Creator</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.BehaviourCreator
	 * @generated
	 */
	public Adapter createBehaviourCreatorAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.ConditionCreator <em>Condition Creator</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.ConditionCreator
	 * @generated
	 */
	public Adapter createConditionCreatorAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.UtilityFunctionCreator <em>Utility Function Creator</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.UtilityFunctionCreator
	 * @generated
	 */
	public Adapter createUtilityFunctionCreatorAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for an object of class '{@link de.uni_kassel.vs.cn.planDesigner.alica.ConstraintCreator <em>Constraint Creator</em>}'.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null so that we can easily ignore cases;
	 * it's useful to ignore a case when inheritance will catch all the cases anyway.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.ConstraintCreator
	 * @generated
	 */
	public Adapter createConstraintCreatorAdapter() {
		return null;
	}

	/**
	 * Creates a new adapter for the default case.
	 * <!-- begin-user-doc -->
	 * This default implementation returns null.
	 * <!-- end-user-doc -->
	 * @return the new adapter.
	 * @generated
	 */
	public Adapter createEObjectAdapter() {
		return null;
	}

} //AlicaAdapterFactory
