/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica.impl;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.ConstraintCreator;

import java.util.Collection;

import org.eclipse.emf.common.util.EList;

import org.eclipse.emf.ecore.EClass;

import org.eclipse.emf.ecore.impl.MinimalEObjectImpl;

import org.eclipse.emf.ecore.util.EObjectResolvingEList;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>Constraint Creator</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.ConstraintCreatorImpl#getConditions <em>Conditions</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.ConstraintCreatorImpl#getPlans <em>Plans</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class ConstraintCreatorImpl extends MinimalEObjectImpl.Container implements ConstraintCreator {
	/**
	 * The cached value of the '{@link #getConditions() <em>Conditions</em>}' reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getConditions()
	 * @generated
	 * @ordered
	 */
	protected EList<Condition> conditions;

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
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected ConstraintCreatorImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return AlicaPackage.Literals.CONSTRAINT_CREATOR;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<Condition> getConditions() {
		if (conditions == null) {
			conditions = new EObjectResolvingEList<Condition>(Condition.class, this, AlicaPackage.CONSTRAINT_CREATOR__CONDITIONS);
		}
		return conditions;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<AbstractPlan> getPlans() {
		if (plans == null) {
			plans = new EObjectResolvingEList<AbstractPlan>(AbstractPlan.class, this, AlicaPackage.CONSTRAINT_CREATOR__PLANS);
		}
		return plans;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public Object eGet(int featureID, boolean resolve, boolean coreType) {
		switch (featureID) {
			case AlicaPackage.CONSTRAINT_CREATOR__CONDITIONS:
				return getConditions();
			case AlicaPackage.CONSTRAINT_CREATOR__PLANS:
				return getPlans();
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
			case AlicaPackage.CONSTRAINT_CREATOR__CONDITIONS:
				getConditions().clear();
				getConditions().addAll((Collection<? extends Condition>)newValue);
				return;
			case AlicaPackage.CONSTRAINT_CREATOR__PLANS:
				getPlans().clear();
				getPlans().addAll((Collection<? extends AbstractPlan>)newValue);
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
			case AlicaPackage.CONSTRAINT_CREATOR__CONDITIONS:
				getConditions().clear();
				return;
			case AlicaPackage.CONSTRAINT_CREATOR__PLANS:
				getPlans().clear();
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
			case AlicaPackage.CONSTRAINT_CREATOR__CONDITIONS:
				return conditions != null && !conditions.isEmpty();
			case AlicaPackage.CONSTRAINT_CREATOR__PLANS:
				return plans != null && !plans.isEmpty();
		}
		return super.eIsSet(featureID);
	}

} //ConstraintCreatorImpl
