/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica.impl;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.InternalEObject;
import org.eclipse.emf.ecore.impl.ENotificationImpl;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Variable;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>Parametrisation</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.ParametrisationImpl#getSubplan <em>Subplan</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.ParametrisationImpl#getSubvar <em>Subvar</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.ParametrisationImpl#getVar <em>Var</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class ParametrisationImpl extends PlanElementImpl implements Parametrisation {
	/**
	 * The cached value of the '{@link #getSubplan() <em>Subplan</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getSubplan()
	 * @generated
	 * @ordered
	 */
	protected AbstractPlan subplan;

	/**
	 * The cached value of the '{@link #getSubvar() <em>Subvar</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getSubvar()
	 * @generated
	 * @ordered
	 */
	protected Variable subvar;

	/**
	 * The cached value of the '{@link #getVar() <em>Var</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getVar()
	 * @generated
	 * @ordered
	 */
	protected Variable var;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected ParametrisationImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return AlicaPackage.Literals.PARAMETRISATION;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public AbstractPlan getSubplan() {
		if (subplan != null && subplan.eIsProxy()) {
			InternalEObject oldSubplan = (InternalEObject)subplan;
			subplan = (AbstractPlan)eResolveProxy(oldSubplan);
			if (subplan != oldSubplan) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.PARAMETRISATION__SUBPLAN, oldSubplan, subplan));
			}
		}
		return subplan;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public AbstractPlan basicGetSubplan() {
		return subplan;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setSubplan(AbstractPlan newSubplan) {
		AbstractPlan oldSubplan = subplan;
		subplan = newSubplan;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PARAMETRISATION__SUBPLAN, oldSubplan, subplan));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Variable getSubvar() {
		if (subvar != null && subvar.eIsProxy()) {
			InternalEObject oldSubvar = (InternalEObject)subvar;
			subvar = (Variable)eResolveProxy(oldSubvar);
			if (subvar != oldSubvar) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.PARAMETRISATION__SUBVAR, oldSubvar, subvar));
			}
		}
		return subvar;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Variable basicGetSubvar() {
		return subvar;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setSubvar(Variable newSubvar) {
		Variable oldSubvar = subvar;
		subvar = newSubvar;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PARAMETRISATION__SUBVAR, oldSubvar, subvar));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Variable getVar() {
		if (var != null && var.eIsProxy()) {
			InternalEObject oldVar = (InternalEObject)var;
			var = (Variable)eResolveProxy(oldVar);
			if (var != oldVar) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.PARAMETRISATION__VAR, oldVar, var));
			}
		}
		return var;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Variable basicGetVar() {
		return var;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setVar(Variable newVar) {
		Variable oldVar = var;
		var = newVar;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.PARAMETRISATION__VAR, oldVar, var));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public Object eGet(int featureID, boolean resolve, boolean coreType) {
		switch (featureID) {
			case AlicaPackage.PARAMETRISATION__SUBPLAN:
				if (resolve) return getSubplan();
				return basicGetSubplan();
			case AlicaPackage.PARAMETRISATION__SUBVAR:
				if (resolve) return getSubvar();
				return basicGetSubvar();
			case AlicaPackage.PARAMETRISATION__VAR:
				if (resolve) return getVar();
				return basicGetVar();
		}
		return super.eGet(featureID, resolve, coreType);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public void eSet(int featureID, Object newValue) {
		switch (featureID) {
			case AlicaPackage.PARAMETRISATION__SUBPLAN:
				setSubplan((AbstractPlan)newValue);
				return;
			case AlicaPackage.PARAMETRISATION__SUBVAR:
				setSubvar((Variable)newValue);
				return;
			case AlicaPackage.PARAMETRISATION__VAR:
				setVar((Variable)newValue);
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
			case AlicaPackage.PARAMETRISATION__SUBPLAN:
				setSubplan((AbstractPlan)null);
				return;
			case AlicaPackage.PARAMETRISATION__SUBVAR:
				setSubvar((Variable)null);
				return;
			case AlicaPackage.PARAMETRISATION__VAR:
				setVar((Variable)null);
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
			case AlicaPackage.PARAMETRISATION__SUBPLAN:
				return subplan != null;
			case AlicaPackage.PARAMETRISATION__SUBVAR:
				return subvar != null;
			case AlicaPackage.PARAMETRISATION__VAR:
				return var != null;
		}
		return super.eIsSet(featureID);
	}

} //ParametrisationImpl
