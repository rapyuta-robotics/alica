/**
 */
package de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl;

import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap;

import org.eclipse.emf.common.notify.NotificationChain;

import org.eclipse.emf.common.util.EMap;

import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.emf.ecore.InternalEObject;

import org.eclipse.emf.ecore.impl.EObjectImpl;

import org.eclipse.emf.ecore.util.EcoreEMap;
import org.eclipse.emf.ecore.util.InternalEList;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>Pml Ui Extension Map</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUiExtensionMapImpl#getExtension <em>Extension</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class PmlUiExtensionMapImpl extends EObjectImpl implements PmlUiExtensionMap {
	/**
	 * The cached value of the '{@link #getExtension() <em>Extension</em>}' map.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getExtension()
	 * @generated
	 * @ordered
	 */
	protected EMap<EObject, PmlUiExtension> extension;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected PmlUiExtensionMapImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return PmlUIExtensionModelPackage.Literals.PML_UI_EXTENSION_MAP;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EMap<EObject, PmlUiExtension> getExtension() {
		if (extension == null) {
			extension = new EcoreEMap<EObject,PmlUiExtension>(PmlUIExtensionModelPackage.Literals.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY, EObjectToPmlUiExtensionMapEntryImpl.class, this, PmlUIExtensionModelPackage.PML_UI_EXTENSION_MAP__EXTENSION);
		}
		return extension;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public NotificationChain eInverseRemove(InternalEObject otherEnd, int featureID, NotificationChain msgs) {
		switch (featureID) {
			case PmlUIExtensionModelPackage.PML_UI_EXTENSION_MAP__EXTENSION:
				return ((InternalEList<?>)getExtension()).basicRemove(otherEnd, msgs);
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
			case PmlUIExtensionModelPackage.PML_UI_EXTENSION_MAP__EXTENSION:
				if (coreType) return getExtension();
				else return getExtension().map();
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
			case PmlUIExtensionModelPackage.PML_UI_EXTENSION_MAP__EXTENSION:
				((EStructuralFeature.Setting)getExtension()).set(newValue);
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
			case PmlUIExtensionModelPackage.PML_UI_EXTENSION_MAP__EXTENSION:
				getExtension().clear();
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
			case PmlUIExtensionModelPackage.PML_UI_EXTENSION_MAP__EXTENSION:
				return extension != null && !extension.isEmpty();
		}
		return super.eIsSet(featureID);
	}

} //PmlUiExtensionMapImpl
