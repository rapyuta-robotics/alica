/**
 */
package de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel;

import org.eclipse.emf.ecore.EFactory;

/**
 * <!-- begin-user-doc -->
 * The <b>Factory</b> for the model.
 * It provides a create method for each non-abstract class of the model.
 * <!-- end-user-doc -->
 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage
 * @generated
 */
public interface PmlUIExtensionModelFactory extends EFactory {
	/**
	 * The singleton instance of the factory.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	PmlUIExtensionModelFactory eINSTANCE = de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelFactoryImpl.init();

	/**
	 * Returns a new object of class '<em>Pml Ui Extension</em>'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return a new object of class '<em>Pml Ui Extension</em>'.
	 * @generated
	 */
	PmlUiExtension createPmlUiExtension();

	/**
	 * Returns a new object of class '<em>Pml Ui Extension Map</em>'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return a new object of class '<em>Pml Ui Extension Map</em>'.
	 * @generated
	 */
	PmlUiExtensionMap createPmlUiExtensionMap();

	/**
	 * Returns a new object of class '<em>Bendpoint</em>'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return a new object of class '<em>Bendpoint</em>'.
	 * @generated
	 */
	Bendpoint createBendpoint();

	/**
	 * Returns the package supported by this factory.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the package supported by this factory.
	 * @generated
	 */
	PmlUIExtensionModelPackage getPmlUIExtensionModelPackage();

} //PmlUIExtensionModelFactory
