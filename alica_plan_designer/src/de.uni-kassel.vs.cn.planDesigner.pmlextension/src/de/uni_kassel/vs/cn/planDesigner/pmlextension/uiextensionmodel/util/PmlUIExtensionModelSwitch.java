/**
 */
package de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.util;

import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.*;

import java.util.List;
import java.util.Map;

import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.EObject;

/**
 * <!-- begin-user-doc -->
 * The <b>Switch</b> for the model's inheritance hierarchy.
 * It supports the call {@link #doSwitch(EObject) doSwitch(object)}
 * to invoke the <code>caseXXX</code> method for each class of the model,
 * starting with the actual class of the object
 * and proceeding up the inheritance hierarchy
 * until a non-null result is returned,
 * which is the result of the switch.
 * <!-- end-user-doc -->
 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage
 * @generated
 */
public class PmlUIExtensionModelSwitch<T> {
	/**
	 * The cached model package
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected static PmlUIExtensionModelPackage modelPackage;

	/**
	 * Creates an instance of the switch.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PmlUIExtensionModelSwitch() {
		if (modelPackage == null) {
			modelPackage = PmlUIExtensionModelPackage.eINSTANCE;
		}
	}

	/**
	 * Calls <code>caseXXX</code> for each class of the model until one returns a non null result; it yields that result.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the first non-null result returned by a <code>caseXXX</code> call.
	 * @generated
	 */
	public T doSwitch(EObject theEObject) {
		return doSwitch(theEObject.eClass(), theEObject);
	}

	/**
	 * Calls <code>caseXXX</code> for each class of the model until one returns a non null result; it yields that result.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the first non-null result returned by a <code>caseXXX</code> call.
	 * @generated
	 */
	protected T doSwitch(EClass theEClass, EObject theEObject) {
		if (theEClass.eContainer() == modelPackage) {
			return doSwitch(theEClass.getClassifierID(), theEObject);
		}
		else {
			List<EClass> eSuperTypes = theEClass.getESuperTypes();
			return
				eSuperTypes.isEmpty() ?
					defaultCase(theEObject) :
					doSwitch(eSuperTypes.get(0), theEObject);
		}
	}

	/**
	 * Calls <code>caseXXX</code> for each class of the model until one returns a non null result; it yields that result.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the first non-null result returned by a <code>caseXXX</code> call.
	 * @generated
	 */
	protected T doSwitch(int classifierID, EObject theEObject) {
		switch (classifierID) {
			case PmlUIExtensionModelPackage.PML_UI_EXTENSION: {
				PmlUiExtension pmlUiExtension = (PmlUiExtension)theEObject;
				T result = casePmlUiExtension(pmlUiExtension);
				if (result == null) result = defaultCase(theEObject);
				return result;
			}
			case PmlUIExtensionModelPackage.PML_UI_EXTENSION_MAP: {
				PmlUiExtensionMap pmlUiExtensionMap = (PmlUiExtensionMap)theEObject;
				T result = casePmlUiExtensionMap(pmlUiExtensionMap);
				if (result == null) result = defaultCase(theEObject);
				return result;
			}
			case PmlUIExtensionModelPackage.EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY: {
				@SuppressWarnings("unchecked") Map.Entry<EObject, PmlUiExtension> eObjectToPmlUiExtensionMapEntry = (Map.Entry<EObject, PmlUiExtension>)theEObject;
				T result = caseEObjectToPmlUiExtensionMapEntry(eObjectToPmlUiExtensionMapEntry);
				if (result == null) result = defaultCase(theEObject);
				return result;
			}
			case PmlUIExtensionModelPackage.BENDPOINT: {
				Bendpoint bendpoint = (Bendpoint)theEObject;
				T result = caseBendpoint(bendpoint);
				if (result == null) result = defaultCase(theEObject);
				return result;
			}
			default: return defaultCase(theEObject);
		}
	}

	/**
	 * Returns the result of interpreting the object as an instance of '<em>Pml Ui Extension</em>'.
	 * <!-- begin-user-doc -->
	 * This implementation returns null;
	 * returning a non-null result will terminate the switch.
	 * <!-- end-user-doc -->
	 * @param object the target of the switch.
	 * @return the result of interpreting the object as an instance of '<em>Pml Ui Extension</em>'.
	 * @see #doSwitch(org.eclipse.emf.ecore.EObject) doSwitch(EObject)
	 * @generated
	 */
	public T casePmlUiExtension(PmlUiExtension object) {
		return null;
	}

	/**
	 * Returns the result of interpreting the object as an instance of '<em>Pml Ui Extension Map</em>'.
	 * <!-- begin-user-doc -->
	 * This implementation returns null;
	 * returning a non-null result will terminate the switch.
	 * <!-- end-user-doc -->
	 * @param object the target of the switch.
	 * @return the result of interpreting the object as an instance of '<em>Pml Ui Extension Map</em>'.
	 * @see #doSwitch(org.eclipse.emf.ecore.EObject) doSwitch(EObject)
	 * @generated
	 */
	public T casePmlUiExtensionMap(PmlUiExtensionMap object) {
		return null;
	}

	/**
	 * Returns the result of interpreting the object as an instance of '<em>EObject To Pml Ui Extension Map Entry</em>'.
	 * <!-- begin-user-doc -->
	 * This implementation returns null;
	 * returning a non-null result will terminate the switch.
	 * <!-- end-user-doc -->
	 * @param object the target of the switch.
	 * @return the result of interpreting the object as an instance of '<em>EObject To Pml Ui Extension Map Entry</em>'.
	 * @see #doSwitch(org.eclipse.emf.ecore.EObject) doSwitch(EObject)
	 * @generated
	 */
	public T caseEObjectToPmlUiExtensionMapEntry(Map.Entry<EObject, PmlUiExtension> object) {
		return null;
	}

	/**
	 * Returns the result of interpreting the object as an instance of '<em>Bendpoint</em>'.
	 * <!-- begin-user-doc -->
	 * This implementation returns null;
	 * returning a non-null result will terminate the switch.
	 * <!-- end-user-doc -->
	 * @param object the target of the switch.
	 * @return the result of interpreting the object as an instance of '<em>Bendpoint</em>'.
	 * @see #doSwitch(org.eclipse.emf.ecore.EObject) doSwitch(EObject)
	 * @generated
	 */
	public T caseBendpoint(Bendpoint object) {
		return null;
	}

	/**
	 * Returns the result of interpreting the object as an instance of '<em>EObject</em>'.
	 * <!-- begin-user-doc -->
	 * This implementation returns null;
	 * returning a non-null result will terminate the switch, but this is the last case anyway.
	 * <!-- end-user-doc -->
	 * @param object the target of the switch.
	 * @return the result of interpreting the object as an instance of '<em>EObject</em>'.
	 * @see #doSwitch(org.eclipse.emf.ecore.EObject)
	 * @generated
	 */
	public T defaultCase(EObject object) {
		return null;
	}

} //PmlUIExtensionModelSwitch
