/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica.impl;

import java.util.Collection;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.InternalEObject;
import org.eclipse.emf.ecore.impl.ENotificationImpl;
import org.eclipse.emf.ecore.util.EObjectResolvingEList;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Planner;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningType;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>Planning Problem</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.PlanningProblemImpl#getPlans <em>Plans</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.PlanningProblemImpl#getPlanner <em>Planner</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.PlanningProblemImpl#getAlternativePlan <em>Alternative Plan</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.PlanningProblemImpl#getWaitPlan <em>Wait Plan</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.PlanningProblemImpl#getUpdateRate <em>Update Rate</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.PlanningProblemImpl#isDistributeProblem <em>Distribute Problem</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.PlanningProblemImpl#getPlanningType <em>Planning Type</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.PlanningProblemImpl#getRequirements <em>Requirements</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.PlanningProblemImpl#getPlannerParams <em>Planner Params</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class PlanningProblemImpl extends AbstractPlanImpl implements PlanningProblem {
	/**
	 * The cached value of the '{@link #getPlans() <em>Plans</em>}' reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getPlans()
	 * @generated
	 * @ordered
	 */
	protected EList<AbstractPlan> plans;

	/**
	 * The cached value of the '{@link #getPlanner() <em>Planner</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getPlanner()
	 * @generated
	 * @ordered
	 */
	protected Planner planner;
	/**
	 * The cached value of the '{@link #getAlternativePlan() <em>Alternative Plan</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getAlternativePlan()
	 * @generated
	 * @ordered
	 */
	protected AbstractPlan alternativePlan;

	/**
	 * The cached value of the '{@link #getWaitPlan() <em>Wait Plan</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getWaitPlan()
	 * @generated
	 * @ordered
	 */
	protected AbstractPlan waitPlan;

	/**
	 * The default value of the '{@link #getUpdateRate() <em>Update Rate</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getUpdateRate()
	 * @generated
	 * @ordered
	 */
	protected static final int UPDATE_RATE_EDEFAULT = 0;

	/**
	 * The cached value of the '{@link #getUpdateRate() <em>Update Rate</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getUpdateRate()
	 * @generated
	 * @ordered
	 */
	protected int updateRate = UPDATE_RATE_EDEFAULT;

	/**
	 * The default value of the '{@link #isDistributeProblem() <em>Distribute Problem</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #isDistributeProblem()
	 * @generated
	 * @ordered
	 */
	protected static final boolean DISTRIBUTE_PROBLEM_EDEFAULT = false;

	/**
	 * The cached value of the '{@link #isDistributeProblem() <em>Distribute Problem</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #isDistributeProblem()
	 * @generated
	 * @ordered
	 */
	protected boolean distributeProblem = DISTRIBUTE_PROBLEM_EDEFAULT;

	/**
	 * The default value of the '{@link #getPlanningType() <em>Planning Type</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getPlanningType()
	 * @generated
	 * @ordered
	 */
	protected static final PlanningType PLANNING_TYPE_EDEFAULT = PlanningType.ONLINE;

	/**
	 * The cached value of the '{@link #getPlanningType() <em>Planning Type</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getPlanningType()
	 * @generated
	 * @ordered
	 */
	protected PlanningType planningType = PLANNING_TYPE_EDEFAULT;

	/**
	 * The default value of the '{@link #getRequirements() <em>Requirements</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getRequirements()
	 * @generated
	 * @ordered
	 */
	protected static final String REQUIREMENTS_EDEFAULT = null;

	/**
	 * The cached value of the '{@link #getRequirements() <em>Requirements</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getRequirements()
	 * @generated
	 * @ordered
	 */
	protected String requirements = REQUIREMENTS_EDEFAULT;

	/**
	 * The default value of the '{@link #getPlannerParams() <em>Planner Params</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getPlannerParams()
	 * @generated
	 * @ordered
	 */
	protected static final String PLANNER_PARAMS_EDEFAULT = null;

	/**
	 * The cached value of the '{@link #getPlannerParams() <em>Planner Params</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getPlannerParams()
	 * @generated
	 * @ordered
	 */
	protected String plannerParams = PLANNER_PARAMS_EDEFAULT;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected PlanningProblemImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return AlicaPackage.Literals.PLANNING_PROBLEM;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<AbstractPlan> getPlans() {
		if (plans == null) {
			plans = new EObjectResolvingEList<AbstractPlan>(AbstractPlan.class, this, AlicaPackage.PLANNING_PROBLEM__PLANS);
		}
		return plans;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Planner getPlanner() {
		if (planner != null && planner.eIsProxy()) {
			InternalEObject oldPlanner = (InternalEObject)planner;
			planner = (Planner)eResolveProxy(oldPlanner);
			if (planner != oldPlanner) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.PLANNING_PROBLEM__PLANNER, oldPlanner, planner));
			}
		}
		return planner;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Planner basicGetPlanner() {
		return planner;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setPlanner(Planner newPlanner) {
		Planner oldPlanner = planner;
		planner = newPlanner;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PLANNING_PROBLEM__PLANNER, oldPlanner, planner));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public AbstractPlan getAlternativePlan() {
		if (alternativePlan != null && alternativePlan.eIsProxy()) {
			InternalEObject oldAlternativePlan = (InternalEObject)alternativePlan;
			alternativePlan = (AbstractPlan)eResolveProxy(oldAlternativePlan);
			if (alternativePlan != oldAlternativePlan) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.PLANNING_PROBLEM__ALTERNATIVE_PLAN, oldAlternativePlan, alternativePlan));
			}
		}
		return alternativePlan;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public AbstractPlan basicGetAlternativePlan() {
		return alternativePlan;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setAlternativePlan(AbstractPlan newAlternativePlan) {
		AbstractPlan oldAlternativePlan = alternativePlan;
		alternativePlan = newAlternativePlan;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PLANNING_PROBLEM__ALTERNATIVE_PLAN, oldAlternativePlan, alternativePlan));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public AbstractPlan getWaitPlan() {
		if (waitPlan != null && waitPlan.eIsProxy()) {
			InternalEObject oldWaitPlan = (InternalEObject)waitPlan;
			waitPlan = (AbstractPlan)eResolveProxy(oldWaitPlan);
			if (waitPlan != oldWaitPlan) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.PLANNING_PROBLEM__WAIT_PLAN, oldWaitPlan, waitPlan));
			}
		}
		return waitPlan;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public AbstractPlan basicGetWaitPlan() {
		return waitPlan;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setWaitPlan(AbstractPlan newWaitPlan) {
		AbstractPlan oldWaitPlan = waitPlan;
		waitPlan = newWaitPlan;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PLANNING_PROBLEM__WAIT_PLAN, oldWaitPlan, waitPlan));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public int getUpdateRate() {
		return updateRate;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setUpdateRate(int newUpdateRate) {
		int oldUpdateRate = updateRate;
		updateRate = newUpdateRate;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PLANNING_PROBLEM__UPDATE_RATE, oldUpdateRate, updateRate));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public boolean isDistributeProblem() {
		return distributeProblem;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setDistributeProblem(boolean newDistributeProblem) {
		boolean oldDistributeProblem = distributeProblem;
		distributeProblem = newDistributeProblem;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PLANNING_PROBLEM__DISTRIBUTE_PROBLEM, oldDistributeProblem, distributeProblem));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PlanningType getPlanningType() {
		return planningType;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setPlanningType(PlanningType newPlanningType) {
		PlanningType oldPlanningType = planningType;
		planningType = newPlanningType == null ? PLANNING_TYPE_EDEFAULT : newPlanningType;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PLANNING_PROBLEM__PLANNING_TYPE, oldPlanningType, planningType));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public String getRequirements() {
		return requirements;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setRequirements(String newRequirements) {
		String oldRequirements = requirements;
		requirements = newRequirements;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PLANNING_PROBLEM__REQUIREMENTS, oldRequirements, requirements));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public String getPlannerParams() {
		return plannerParams;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setPlannerParams(String newPlannerParams) {
		String oldPlannerParams = plannerParams;
		plannerParams = newPlannerParams;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PLANNING_PROBLEM__PLANNER_PARAMS, oldPlannerParams, plannerParams));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public Object eGet(int featureID, boolean resolve, boolean coreType) {
		switch (featureID) {
			case AlicaPackage.PLANNING_PROBLEM__PLANS:
				return getPlans();
			case AlicaPackage.PLANNING_PROBLEM__PLANNER:
				if (resolve) return getPlanner();
				return basicGetPlanner();
			case AlicaPackage.PLANNING_PROBLEM__ALTERNATIVE_PLAN:
				if (resolve) return getAlternativePlan();
				return basicGetAlternativePlan();
			case AlicaPackage.PLANNING_PROBLEM__WAIT_PLAN:
				if (resolve) return getWaitPlan();
				return basicGetWaitPlan();
			case AlicaPackage.PLANNING_PROBLEM__UPDATE_RATE:
				return getUpdateRate();
			case AlicaPackage.PLANNING_PROBLEM__DISTRIBUTE_PROBLEM:
				return isDistributeProblem();
			case AlicaPackage.PLANNING_PROBLEM__PLANNING_TYPE:
				return getPlanningType();
			case AlicaPackage.PLANNING_PROBLEM__REQUIREMENTS:
				return getRequirements();
			case AlicaPackage.PLANNING_PROBLEM__PLANNER_PARAMS:
				return getPlannerParams();
		}
		return super.eGet(featureID, resolve, coreType);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@SuppressWarnings("unchecked")
	@Override
	public void eSet(int featureID, Object newValue) {
		switch (featureID) {
			case AlicaPackage.PLANNING_PROBLEM__PLANS:
				getPlans().clear();
				getPlans().addAll((Collection<? extends AbstractPlan>)newValue);
				return;
			case AlicaPackage.PLANNING_PROBLEM__PLANNER:
				setPlanner((Planner)newValue);
				return;
			case AlicaPackage.PLANNING_PROBLEM__ALTERNATIVE_PLAN:
				setAlternativePlan((AbstractPlan)newValue);
				return;
			case AlicaPackage.PLANNING_PROBLEM__WAIT_PLAN:
				setWaitPlan((AbstractPlan)newValue);
				return;
			case AlicaPackage.PLANNING_PROBLEM__UPDATE_RATE:
				setUpdateRate((Integer)newValue);
				return;
			case AlicaPackage.PLANNING_PROBLEM__DISTRIBUTE_PROBLEM:
				setDistributeProblem((Boolean)newValue);
				return;
			case AlicaPackage.PLANNING_PROBLEM__PLANNING_TYPE:
				setPlanningType((PlanningType)newValue);
				return;
			case AlicaPackage.PLANNING_PROBLEM__REQUIREMENTS:
				setRequirements((String)newValue);
				return;
			case AlicaPackage.PLANNING_PROBLEM__PLANNER_PARAMS:
				setPlannerParams((String)newValue);
				return;
		}
		super.eSet(featureID, newValue);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public void eUnset(int featureID) {
		switch (featureID) {
			case AlicaPackage.PLANNING_PROBLEM__PLANS:
				getPlans().clear();
				return;
			case AlicaPackage.PLANNING_PROBLEM__PLANNER:
				setPlanner((Planner)null);
				return;
			case AlicaPackage.PLANNING_PROBLEM__ALTERNATIVE_PLAN:
				setAlternativePlan((AbstractPlan)null);
				return;
			case AlicaPackage.PLANNING_PROBLEM__WAIT_PLAN:
				setWaitPlan((AbstractPlan)null);
				return;
			case AlicaPackage.PLANNING_PROBLEM__UPDATE_RATE:
				setUpdateRate(UPDATE_RATE_EDEFAULT);
				return;
			case AlicaPackage.PLANNING_PROBLEM__DISTRIBUTE_PROBLEM:
				setDistributeProblem(DISTRIBUTE_PROBLEM_EDEFAULT);
				return;
			case AlicaPackage.PLANNING_PROBLEM__PLANNING_TYPE:
				setPlanningType(PLANNING_TYPE_EDEFAULT);
				return;
			case AlicaPackage.PLANNING_PROBLEM__REQUIREMENTS:
				setRequirements(REQUIREMENTS_EDEFAULT);
				return;
			case AlicaPackage.PLANNING_PROBLEM__PLANNER_PARAMS:
				setPlannerParams(PLANNER_PARAMS_EDEFAULT);
				return;
		}
		super.eUnset(featureID);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public boolean eIsSet(int featureID) {
		switch (featureID) {
			case AlicaPackage.PLANNING_PROBLEM__PLANS:
				return plans != null && !plans.isEmpty();
			case AlicaPackage.PLANNING_PROBLEM__PLANNER:
				return planner != null;
			case AlicaPackage.PLANNING_PROBLEM__ALTERNATIVE_PLAN:
				return alternativePlan != null;
			case AlicaPackage.PLANNING_PROBLEM__WAIT_PLAN:
				return waitPlan != null;
			case AlicaPackage.PLANNING_PROBLEM__UPDATE_RATE:
				return updateRate != UPDATE_RATE_EDEFAULT;
			case AlicaPackage.PLANNING_PROBLEM__DISTRIBUTE_PROBLEM:
				return distributeProblem != DISTRIBUTE_PROBLEM_EDEFAULT;
			case AlicaPackage.PLANNING_PROBLEM__PLANNING_TYPE:
				return planningType != PLANNING_TYPE_EDEFAULT;
			case AlicaPackage.PLANNING_PROBLEM__REQUIREMENTS:
				return REQUIREMENTS_EDEFAULT == null ? requirements != null : !REQUIREMENTS_EDEFAULT.equals(requirements);
			case AlicaPackage.PLANNING_PROBLEM__PLANNER_PARAMS:
				return PLANNER_PARAMS_EDEFAULT == null ? plannerParams != null : !PLANNER_PARAMS_EDEFAULT.equals(plannerParams);
		}
		return super.eIsSet(featureID);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public String toString() {
		if (eIsProxy()) return super.toString();

		StringBuffer result = new StringBuffer(super.toString());
		result.append(" (updateRate: ");
		result.append(updateRate);
		result.append(", distributeProblem: ");
		result.append(distributeProblem);
		result.append(", planningType: ");
		result.append(planningType);
		result.append(", requirements: ");
		result.append(requirements);
		result.append(", plannerParams: ");
		result.append(plannerParams);
		result.append(')');
		return result.toString();
	}

} //PlanningProblemImpl
