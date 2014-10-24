/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica.impl;

import java.util.Collection;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.notify.NotificationChain;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.InternalEObject;
import org.eclipse.emf.ecore.impl.ENotificationImpl;
import org.eclipse.emf.ecore.util.EObjectContainmentEList;
import org.eclipse.emf.ecore.util.EObjectContainmentWithInverseEList;
import org.eclipse.emf.ecore.util.InternalEList;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Rating;
import de.uni_kassel.vs.cn.planDesigner.alica.Variable;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>Abstract Plan</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.AbstractPlanImpl#getRating <em>Rating</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.AbstractPlanImpl#getConditions <em>Conditions</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.AbstractPlanImpl#isMasterPlan <em>Master Plan</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.AbstractPlanImpl#getUtilityFunction <em>Utility Function</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.AbstractPlanImpl#getUtilityThreshold <em>Utility Threshold</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.AbstractPlanImpl#getVars <em>Vars</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.AbstractPlanImpl#getDestinationPath <em>Destination Path</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public abstract class AbstractPlanImpl extends PlanElementImpl implements AbstractPlan {
	/**
	 * The cached value of the '{@link #getRating() <em>Rating</em>}' containment reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getRating()
	 * @generated
	 * @ordered
	 */
	protected Rating rating;

	/**
	 * The cached value of the '{@link #getConditions() <em>Conditions</em>}' containment reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getConditions()
	 * @generated
	 * @ordered
	 */
	protected EList<Condition> conditions;

	/**
	 * The default value of the '{@link #isMasterPlan() <em>Master Plan</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #isMasterPlan()
	 * @generated
	 * @ordered
	 */
	protected static final boolean MASTER_PLAN_EDEFAULT = false;

	/**
	 * The cached value of the '{@link #isMasterPlan() <em>Master Plan</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #isMasterPlan()
	 * @generated
	 * @ordered
	 */
	protected boolean masterPlan = MASTER_PLAN_EDEFAULT;

	/**
	 * The default value of the '{@link #getUtilityFunction() <em>Utility Function</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getUtilityFunction()
	 * @generated
	 * @ordered
	 */
	protected static final String UTILITY_FUNCTION_EDEFAULT = "";

	/**
	 * The cached value of the '{@link #getUtilityFunction() <em>Utility Function</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getUtilityFunction()
	 * @generated
	 * @ordered
	 */
	protected String utilityFunction = UTILITY_FUNCTION_EDEFAULT;

	/**
	 * The default value of the '{@link #getUtilityThreshold() <em>Utility Threshold</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getUtilityThreshold()
	 * @generated
	 * @ordered
	 */
	protected static final double UTILITY_THRESHOLD_EDEFAULT = 0.1;

	/**
	 * The cached value of the '{@link #getUtilityThreshold() <em>Utility Threshold</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getUtilityThreshold()
	 * @generated
	 * @ordered
	 */
	protected double utilityThreshold = UTILITY_THRESHOLD_EDEFAULT;

	/**
	 * The cached value of the '{@link #getVars() <em>Vars</em>}' containment reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getVars()
	 * @generated
	 * @ordered
	 */
	protected EList<Variable> vars;

	/**
	 * The default value of the '{@link #getDestinationPath() <em>Destination Path</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getDestinationPath()
	 * @generated
	 * @ordered
	 */
	protected static final String DESTINATION_PATH_EDEFAULT = "";

	/**
	 * The cached value of the '{@link #getDestinationPath() <em>Destination Path</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getDestinationPath()
	 * @generated
	 * @ordered
	 */
	protected String destinationPath = DESTINATION_PATH_EDEFAULT;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected AbstractPlanImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return AlicaPackage.Literals.ABSTRACT_PLAN;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Rating getRating() {
		return rating;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public NotificationChain basicSetRating(Rating newRating, NotificationChain msgs) {
		Rating oldRating = rating;
		rating = newRating;
		if (eNotificationRequired()) {
			ENotificationImpl notification = new ENotificationImpl(this, Notification.SET, AlicaPackage.ABSTRACT_PLAN__RATING, oldRating, newRating);
			if (msgs == null) msgs = notification; else msgs.add(notification);
		}
		return msgs;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setRating(Rating newRating) {
		if (newRating != rating) {
			NotificationChain msgs = null;
			if (rating != null)
				msgs = ((InternalEObject)rating).eInverseRemove(this, EOPPOSITE_FEATURE_BASE - AlicaPackage.ABSTRACT_PLAN__RATING, null, msgs);
			if (newRating != null)
				msgs = ((InternalEObject)newRating).eInverseAdd(this, EOPPOSITE_FEATURE_BASE - AlicaPackage.ABSTRACT_PLAN__RATING, null, msgs);
			msgs = basicSetRating(newRating, msgs);
			if (msgs != null) msgs.dispatch();
		}
		else if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.ABSTRACT_PLAN__RATING, newRating, newRating));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<Condition> getConditions() {
		if (conditions == null) {
			conditions = new EObjectContainmentWithInverseEList<Condition>(Condition.class, this, AlicaPackage.ABSTRACT_PLAN__CONDITIONS, AlicaPackage.CONDITION__ABSTRACT_PLAN);
		}
		return conditions;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public boolean isMasterPlan() {
		return masterPlan;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setMasterPlan(boolean newMasterPlan) {
		boolean oldMasterPlan = masterPlan;
		masterPlan = newMasterPlan;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.ABSTRACT_PLAN__MASTER_PLAN, oldMasterPlan, masterPlan));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public String getUtilityFunction() {
		return utilityFunction;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setUtilityFunction(String newUtilityFunction) {
		String oldUtilityFunction = utilityFunction;
		utilityFunction = newUtilityFunction;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.ABSTRACT_PLAN__UTILITY_FUNCTION, oldUtilityFunction, utilityFunction));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public double getUtilityThreshold() {
		return utilityThreshold;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setUtilityThreshold(double newUtilityThreshold) {
		double oldUtilityThreshold = utilityThreshold;
		utilityThreshold = newUtilityThreshold;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.ABSTRACT_PLAN__UTILITY_THRESHOLD, oldUtilityThreshold, utilityThreshold));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<Variable> getVars() {
		if (vars == null) {
			vars = new EObjectContainmentEList<Variable>(Variable.class, this, AlicaPackage.ABSTRACT_PLAN__VARS);
		}
		return vars;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public String getDestinationPath() {
		return destinationPath;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setDestinationPath(String newDestinationPath) {
		String oldDestinationPath = destinationPath;
		destinationPath = newDestinationPath;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.ABSTRACT_PLAN__DESTINATION_PATH, oldDestinationPath, destinationPath));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@SuppressWarnings("unchecked")
	@Override
	public NotificationChain eInverseAdd(InternalEObject otherEnd, int featureID, NotificationChain msgs) {
		switch (featureID) {
			case AlicaPackage.ABSTRACT_PLAN__CONDITIONS:
				return ((InternalEList<InternalEObject>)(InternalEList<?>)getConditions()).basicAdd(otherEnd, msgs);
		}
		return super.eInverseAdd(otherEnd, featureID, msgs);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public NotificationChain eInverseRemove(InternalEObject otherEnd, int featureID, NotificationChain msgs) {
		switch (featureID) {
			case AlicaPackage.ABSTRACT_PLAN__RATING:
				return basicSetRating(null, msgs);
			case AlicaPackage.ABSTRACT_PLAN__CONDITIONS:
				return ((InternalEList<?>)getConditions()).basicRemove(otherEnd, msgs);
			case AlicaPackage.ABSTRACT_PLAN__VARS:
				return ((InternalEList<?>)getVars()).basicRemove(otherEnd, msgs);
		}
		return super.eInverseRemove(otherEnd, featureID, msgs);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public Object eGet(int featureID, boolean resolve, boolean coreType) {
		switch (featureID) {
			case AlicaPackage.ABSTRACT_PLAN__RATING:
				return getRating();
			case AlicaPackage.ABSTRACT_PLAN__CONDITIONS:
				return getConditions();
			case AlicaPackage.ABSTRACT_PLAN__MASTER_PLAN:
				return isMasterPlan();
			case AlicaPackage.ABSTRACT_PLAN__UTILITY_FUNCTION:
				return getUtilityFunction();
			case AlicaPackage.ABSTRACT_PLAN__UTILITY_THRESHOLD:
				return getUtilityThreshold();
			case AlicaPackage.ABSTRACT_PLAN__VARS:
				return getVars();
			case AlicaPackage.ABSTRACT_PLAN__DESTINATION_PATH:
				return getDestinationPath();
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
			case AlicaPackage.ABSTRACT_PLAN__RATING:
				setRating((Rating)newValue);
				return;
			case AlicaPackage.ABSTRACT_PLAN__CONDITIONS:
				getConditions().clear();
				getConditions().addAll((Collection<? extends Condition>)newValue);
				return;
			case AlicaPackage.ABSTRACT_PLAN__MASTER_PLAN:
				setMasterPlan((Boolean)newValue);
				return;
			case AlicaPackage.ABSTRACT_PLAN__UTILITY_FUNCTION:
				setUtilityFunction((String)newValue);
				return;
			case AlicaPackage.ABSTRACT_PLAN__UTILITY_THRESHOLD:
				setUtilityThreshold((Double)newValue);
				return;
			case AlicaPackage.ABSTRACT_PLAN__VARS:
				getVars().clear();
				getVars().addAll((Collection<? extends Variable>)newValue);
				return;
			case AlicaPackage.ABSTRACT_PLAN__DESTINATION_PATH:
				setDestinationPath((String)newValue);
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
			case AlicaPackage.ABSTRACT_PLAN__RATING:
				setRating((Rating)null);
				return;
			case AlicaPackage.ABSTRACT_PLAN__CONDITIONS:
				getConditions().clear();
				return;
			case AlicaPackage.ABSTRACT_PLAN__MASTER_PLAN:
				setMasterPlan(MASTER_PLAN_EDEFAULT);
				return;
			case AlicaPackage.ABSTRACT_PLAN__UTILITY_FUNCTION:
				setUtilityFunction(UTILITY_FUNCTION_EDEFAULT);
				return;
			case AlicaPackage.ABSTRACT_PLAN__UTILITY_THRESHOLD:
				setUtilityThreshold(UTILITY_THRESHOLD_EDEFAULT);
				return;
			case AlicaPackage.ABSTRACT_PLAN__VARS:
				getVars().clear();
				return;
			case AlicaPackage.ABSTRACT_PLAN__DESTINATION_PATH:
				setDestinationPath(DESTINATION_PATH_EDEFAULT);
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
			case AlicaPackage.ABSTRACT_PLAN__RATING:
				return rating != null;
			case AlicaPackage.ABSTRACT_PLAN__CONDITIONS:
				return conditions != null && !conditions.isEmpty();
			case AlicaPackage.ABSTRACT_PLAN__MASTER_PLAN:
				return masterPlan != MASTER_PLAN_EDEFAULT;
			case AlicaPackage.ABSTRACT_PLAN__UTILITY_FUNCTION:
				return UTILITY_FUNCTION_EDEFAULT == null ? utilityFunction != null : !UTILITY_FUNCTION_EDEFAULT.equals(utilityFunction);
			case AlicaPackage.ABSTRACT_PLAN__UTILITY_THRESHOLD:
				return utilityThreshold != UTILITY_THRESHOLD_EDEFAULT;
			case AlicaPackage.ABSTRACT_PLAN__VARS:
				return vars != null && !vars.isEmpty();
			case AlicaPackage.ABSTRACT_PLAN__DESTINATION_PATH:
				return DESTINATION_PATH_EDEFAULT == null ? destinationPath != null : !DESTINATION_PATH_EDEFAULT.equals(destinationPath);
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
		result.append(" (masterPlan: ");
		result.append(masterPlan);
		result.append(", utilityFunction: ");
		result.append(utilityFunction);
		result.append(", utilityThreshold: ");
		result.append(utilityThreshold);
		result.append(", destinationPath: ");
		result.append(destinationPath);
		result.append(')');
		return result.toString();
	}

} //AbstractPlanImpl
