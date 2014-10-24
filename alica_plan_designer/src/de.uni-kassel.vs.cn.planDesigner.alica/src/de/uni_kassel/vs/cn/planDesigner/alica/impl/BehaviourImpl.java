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
import org.eclipse.emf.ecore.util.EObjectContainmentWithInverseEList;
import org.eclipse.emf.ecore.util.InternalEList;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>Behaviour</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.BehaviourImpl#getConfigurations <em>Configurations</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.BehaviourImpl#getDestinationPath <em>Destination Path</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class BehaviourImpl extends PlanElementImpl implements Behaviour {
	/**
	 * The cached value of the '{@link #getConfigurations() <em>Configurations</em>}' containment reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getConfigurations()
	 * @generated
	 * @ordered
	 */
	protected EList<BehaviourConfiguration> configurations;

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
	protected BehaviourImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return AlicaPackage.Literals.BEHAVIOUR;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<BehaviourConfiguration> getConfigurations() {
		if (configurations == null) {
			configurations = new EObjectContainmentWithInverseEList<BehaviourConfiguration>(BehaviourConfiguration.class, this, AlicaPackage.BEHAVIOUR__CONFIGURATIONS, AlicaPackage.BEHAVIOUR_CONFIGURATION__BEHAVIOUR);
		}
		return configurations;
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
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.BEHAVIOUR__DESTINATION_PATH, oldDestinationPath, destinationPath));
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
			case AlicaPackage.BEHAVIOUR__CONFIGURATIONS:
				return ((InternalEList<InternalEObject>)(InternalEList<?>)getConfigurations()).basicAdd(otherEnd, msgs);
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
			case AlicaPackage.BEHAVIOUR__CONFIGURATIONS:
				return ((InternalEList<?>)getConfigurations()).basicRemove(otherEnd, msgs);
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
			case AlicaPackage.BEHAVIOUR__CONFIGURATIONS:
				return getConfigurations();
			case AlicaPackage.BEHAVIOUR__DESTINATION_PATH:
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
			case AlicaPackage.BEHAVIOUR__CONFIGURATIONS:
				getConfigurations().clear();
				getConfigurations().addAll((Collection<? extends BehaviourConfiguration>)newValue);
				return;
			case AlicaPackage.BEHAVIOUR__DESTINATION_PATH:
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
			case AlicaPackage.BEHAVIOUR__CONFIGURATIONS:
				getConfigurations().clear();
				return;
			case AlicaPackage.BEHAVIOUR__DESTINATION_PATH:
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
			case AlicaPackage.BEHAVIOUR__CONFIGURATIONS:
				return configurations != null && !configurations.isEmpty();
			case AlicaPackage.BEHAVIOUR__DESTINATION_PATH:
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
		result.append(" (destinationPath: ");
		result.append(destinationPath);
		result.append(')');
		return result.toString();
	}

} //BehaviourImpl
