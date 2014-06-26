/**
 */
package de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl;

import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.notify.NotificationChain;

import org.eclipse.emf.common.util.BasicEMap;
import org.eclipse.emf.common.util.EMap;

import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.InternalEObject;

import org.eclipse.emf.ecore.impl.ENotificationImpl;
import org.eclipse.emf.ecore.impl.EObjectImpl;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>EObject To Pml Ui Extension Map Entry</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.EObjectToPmlUiExtensionMapEntryImpl#getTypedKey <em>Key</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.EObjectToPmlUiExtensionMapEntryImpl#getTypedValue <em>Value</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class EObjectToPmlUiExtensionMapEntryImpl extends EObjectImpl implements BasicEMap.Entry<EObject,PmlUiExtension> {
	/**
	 * The cached value of the '{@link #getTypedKey() <em>Key</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getTypedKey()
	 * @generated
	 * @ordered
	 */
	protected EObject key;

	/**
	 * The cached value of the '{@link #getTypedValue() <em>Value</em>}' containment reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getTypedValue()
	 * @generated
	 * @ordered
	 */
	protected PmlUiExtension value;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected EObjectToPmlUiExtensionMapEntryImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return PmlUIExtensionModelPackage.Literals.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EObject getTypedKey() {
		if (key != null && key.eIsProxy()) {
			InternalEObject oldKey = (InternalEObject)key;
			key = eResolveProxy(oldKey);
			if (key != oldKey) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__KEY, oldKey, key));
			}
		}
		return key;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EObject basicGetTypedKey() {
		return key;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setTypedKey(EObject newKey) {
		EObject oldKey = key;
		key = newKey;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__KEY, oldKey, key));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PmlUiExtension getTypedValue() {
		return value;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public NotificationChain basicSetTypedValue(PmlUiExtension newValue, NotificationChain msgs) {
		PmlUiExtension oldValue = value;
		value = newValue;
		if (eNotificationRequired()) {
			ENotificationImpl notification = new ENotificationImpl(this, Notification.SET, PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE, oldValue, newValue);
			if (msgs == null) msgs = notification; else msgs.add(notification);
		}
		return msgs;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setTypedValue(PmlUiExtension newValue) {
		if (newValue != value) {
			NotificationChain msgs = null;
			if (value != null)
				msgs = ((InternalEObject)value).eInverseRemove(this, EOPPOSITE_FEATURE_BASE - PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE, null, msgs);
			if (newValue != null)
				msgs = ((InternalEObject)newValue).eInverseAdd(this, EOPPOSITE_FEATURE_BASE - PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE, null, msgs);
			msgs = basicSetTypedValue(newValue, msgs);
			if (msgs != null) msgs.dispatch();
		}
		else if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE, newValue, newValue));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public NotificationChain eInverseRemove(InternalEObject otherEnd, int featureID, NotificationChain msgs) {
		switch (featureID) {
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE:
				return basicSetTypedValue(null, msgs);
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
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__KEY:
				if (resolve) return getTypedKey();
				return basicGetTypedKey();
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE:
				return getTypedValue();
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
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__KEY:
				setTypedKey((EObject)newValue);
				return;
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE:
				setTypedValue((PmlUiExtension)newValue);
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
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__KEY:
				setTypedKey((EObject)null);
				return;
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE:
				setTypedValue((PmlUiExtension)null);
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
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__KEY:
				return key != null;
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE:
				return value != null;
		}
		return super.eIsSet(featureID);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected int hash = -1;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public int getHash() {
		if (hash == -1) {
			Object theKey = getKey();
			hash = (theKey == null ? 0 : theKey.hashCode());
		}
		return hash;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setHash(int hash) {
		this.hash = hash;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EObject getKey() {
		return getTypedKey();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setKey(EObject key) {
		setTypedKey(key);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PmlUiExtension getValue() {
		return getTypedValue();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PmlUiExtension setValue(PmlUiExtension value) {
		PmlUiExtension oldValue = getValue();
		setTypedValue(value);
		return oldValue;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@SuppressWarnings("unchecked")
	public EMap<EObject, PmlUiExtension> getEMap() {
		EObject container = eContainer();
		return container == null ? null : (EMap<EObject, PmlUiExtension>)container.eGet(eContainmentFeature());
	}

} //EObjectToPmlUiExtensionMapEntryImpl
