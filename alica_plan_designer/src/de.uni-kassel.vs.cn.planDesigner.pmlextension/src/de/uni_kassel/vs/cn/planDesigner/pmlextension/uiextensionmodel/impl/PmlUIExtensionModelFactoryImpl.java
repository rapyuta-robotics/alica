/**
 */
package de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl;

import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.*;

import java.util.Map;

import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EPackage;

import org.eclipse.emf.ecore.impl.EFactoryImpl;

import org.eclipse.emf.ecore.plugin.EcorePlugin;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model <b>Factory</b>.
 * <!-- end-user-doc -->
 * @generated
 */
public class PmlUIExtensionModelFactoryImpl extends EFactoryImpl implements PmlUIExtensionModelFactory {
	/**
	 * Creates the default factory implementation.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public static PmlUIExtensionModelFactory init() {
		try {
			PmlUIExtensionModelFactory thePmlUIExtensionModelFactory = (PmlUIExtensionModelFactory)EPackage.Registry.INSTANCE.getEFactory(PmlUIExtensionModelPackage.eNS_URI);
			if (thePmlUIExtensionModelFactory != null) {
				return thePmlUIExtensionModelFactory;
			}
		}
		catch (Exception exception) {
			EcorePlugin.INSTANCE.log(exception);
		}
		return new PmlUIExtensionModelFactoryImpl();
	}

	/**
	 * Creates an instance of the factory.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PmlUIExtensionModelFactoryImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public EObject create(EClass eClass) {
		switch (eClass.getClassifierID()) {
			case PmlUIExtensionModelPackage.PML_UI_EXTENSION: return createPmlUiExtension();
			case PmlUIExtensionModelPackage.PML_UI_EXTENSION_MAP: return createPmlUiExtensionMap();
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY: return (EObject)createEObjectToPmlUiExtensionMapEntry();
			case PmlUIExtensionModelPackage.BENDPOINT: return createBendpoint();
			default:
				throw new IllegalArgumentException("The class '" + eClass.getName() + "' is not a valid classifier");
		}
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PmlUiExtension createPmlUiExtension() {
		PmlUiExtensionImpl pmlUiExtension = new PmlUiExtensionImpl();
		return pmlUiExtension;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PmlUiExtensionMap createPmlUiExtensionMap() {
		PmlUiExtensionMapImpl pmlUiExtensionMap = new PmlUiExtensionMapImpl();
		return pmlUiExtensionMap;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Map.Entry<EObject, PmlUiExtension> createEObjectToPmlUiExtensionMapEntry() {
		EObjectToPmlUiExtensionMapEntryImpl eObjectToPmlUiExtensionMapEntry = new EObjectToPmlUiExtensionMapEntryImpl();
		return eObjectToPmlUiExtensionMapEntry;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Bendpoint createBendpoint() {
		BendpointImpl bendpoint = new BendpointImpl();
		return bendpoint;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PmlUIExtensionModelPackage getPmlUIExtensionModelPackage() {
		return (PmlUIExtensionModelPackage)getEPackage();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @deprecated
	 * @generated
	 */
	@Deprecated
	public static PmlUIExtensionModelPackage getPackage() {
		return PmlUIExtensionModelPackage.eINSTANCE;
	}

} //PmlUIExtensionModelFactoryImpl
