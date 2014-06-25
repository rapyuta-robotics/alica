/**
 */
package de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel;

import org.eclipse.emf.common.util.EMap;

import org.eclipse.emf.ecore.EObject;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Pml Ui Extension Map</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap#getExtension <em>Extension</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage#getPmlUiExtensionMap()
 * @model
 * @generated
 */
public interface PmlUiExtensionMap extends EObject {
	/**
	 * Returns the value of the '<em><b>Extension</b></em>' map.
	 * The key is of type {@link org.eclipse.emf.ecore.EObject},
	 * and the value is of type {@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension},
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Extension</em>' map isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Extension</em>' map.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage#getPmlUiExtensionMap_Extension()
	 * @model mapType="de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.EObjectToPmlUiExtensionMapEntry<org.eclipse.emf.ecore.EObject, de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension>"
	 *        annotation="http:///org/eclipse/emf/mapping/xsd2ecore/XSD2Ecore name='extensions'"
	 * @generated
	 */
	EMap<EObject, PmlUiExtension> getExtension();

} // PmlUiExtensionMap
