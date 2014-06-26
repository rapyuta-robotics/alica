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
import org.eclipse.emf.ecore.util.InternalEList;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleSet;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>Role Set</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.RoleSetImpl#getUsableWithPlanID <em>Usable With Plan ID</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.RoleSetImpl#isDefault <em>Default</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.RoleSetImpl#getMappings <em>Mappings</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class RoleSetImpl extends PlanElementImpl implements RoleSet {
	/**
	 * The default value of the '{@link #getUsableWithPlanID() <em>Usable With Plan ID</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getUsableWithPlanID()
	 * @generated
	 * @ordered
	 */
	protected static final long USABLE_WITH_PLAN_ID_EDEFAULT = 0L;

	/**
	 * The cached value of the '{@link #getUsableWithPlanID() <em>Usable With Plan ID</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getUsableWithPlanID()
	 * @generated
	 * @ordered
	 */
	protected long usableWithPlanID = USABLE_WITH_PLAN_ID_EDEFAULT;

	/**
	 * The default value of the '{@link #isDefault() <em>Default</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #isDefault()
	 * @generated
	 * @ordered
	 */
	protected static final boolean DEFAULT_EDEFAULT = false;

	/**
	 * The cached value of the '{@link #isDefault() <em>Default</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #isDefault()
	 * @generated
	 * @ordered
	 */
	protected boolean default_ = DEFAULT_EDEFAULT;

	/**
	 * The cached value of the '{@link #getMappings() <em>Mappings</em>}' containment reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getMappings()
	 * @generated
	 * @ordered
	 */
	protected EList<RoleTaskMapping> mappings;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected RoleSetImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return AlicaPackage.Literals.ROLE_SET;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public long getUsableWithPlanID() {
		return usableWithPlanID;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setUsableWithPlanID(long newUsableWithPlanID) {
		long oldUsableWithPlanID = usableWithPlanID;
		usableWithPlanID = newUsableWithPlanID;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.ROLE_SET__USABLE_WITH_PLAN_ID, oldUsableWithPlanID, usableWithPlanID));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public boolean isDefault() {
		return default_;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setDefault(boolean newDefault) {
		boolean oldDefault = default_;
		default_ = newDefault;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.ROLE_SET__DEFAULT, oldDefault, default_));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<RoleTaskMapping> getMappings() {
		if (mappings == null) {
			mappings = new EObjectContainmentEList<RoleTaskMapping>(RoleTaskMapping.class, this, AlicaPackage.ROLE_SET__MAPPINGS);
		}
		return mappings;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public NotificationChain eInverseRemove(InternalEObject otherEnd, int featureID, NotificationChain msgs) {
		switch (featureID) {
			case AlicaPackage.ROLE_SET__MAPPINGS:
				return ((InternalEList<?>)getMappings()).basicRemove(otherEnd, msgs);
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
			case AlicaPackage.ROLE_SET__USABLE_WITH_PLAN_ID:
				return getUsableWithPlanID();
			case AlicaPackage.ROLE_SET__DEFAULT:
				return isDefault();
			case AlicaPackage.ROLE_SET__MAPPINGS:
				return getMappings();
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
			case AlicaPackage.ROLE_SET__USABLE_WITH_PLAN_ID:
				setUsableWithPlanID((Long)newValue);
				return;
			case AlicaPackage.ROLE_SET__DEFAULT:
				setDefault((Boolean)newValue);
				return;
			case AlicaPackage.ROLE_SET__MAPPINGS:
				getMappings().clear();
				getMappings().addAll((Collection<? extends RoleTaskMapping>)newValue);
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
			case AlicaPackage.ROLE_SET__USABLE_WITH_PLAN_ID:
				setUsableWithPlanID(USABLE_WITH_PLAN_ID_EDEFAULT);
				return;
			case AlicaPackage.ROLE_SET__DEFAULT:
				setDefault(DEFAULT_EDEFAULT);
				return;
			case AlicaPackage.ROLE_SET__MAPPINGS:
				getMappings().clear();
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
			case AlicaPackage.ROLE_SET__USABLE_WITH_PLAN_ID:
				return usableWithPlanID != USABLE_WITH_PLAN_ID_EDEFAULT;
			case AlicaPackage.ROLE_SET__DEFAULT:
				return default_ != DEFAULT_EDEFAULT;
			case AlicaPackage.ROLE_SET__MAPPINGS:
				return mappings != null && !mappings.isEmpty();
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
		result.append(" (usableWithPlanID: ");
		result.append(usableWithPlanID);
		result.append(", default: ");
		result.append(default_);
		result.append(')');
		return result.toString();
	}

} //RoleSetImpl
