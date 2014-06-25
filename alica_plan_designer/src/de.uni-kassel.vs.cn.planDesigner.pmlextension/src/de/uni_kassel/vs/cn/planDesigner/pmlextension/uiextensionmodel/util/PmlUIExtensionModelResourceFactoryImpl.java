/**
 */
package de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.util;

import org.eclipse.emf.common.util.URI;

import org.eclipse.emf.ecore.resource.Resource;

import org.eclipse.emf.ecore.resource.impl.ResourceFactoryImpl;

import org.eclipse.emf.ecore.xmi.XMLResource;

import org.eclipse.emf.ecore.xmi.impl.XMLMapImpl;

/**
 * <!-- begin-user-doc -->
 * The <b>Resource Factory</b> associated with the package.
 * <!-- end-user-doc -->
 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.util.PmlUIExtensionModelResourceImpl
 * @generated
 */
public class PmlUIExtensionModelResourceFactoryImpl extends ResourceFactoryImpl {
	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected XMLResource.XMLMap xmlMap = new XMLMapImpl();

	/**
	 * Creates an instance of the resource factory.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PmlUIExtensionModelResourceFactoryImpl() {
		super();
	}

	/**
	 * Creates an instance of the resource.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public Resource createResource(URI uri) {
		XMLResource result = new PmlUIExtensionModelResourceImpl(uri);
		result.getDefaultSaveOptions().put(XMLResource.OPTION_XML_MAP, xmlMap);
		result.getDefaultLoadOptions().put(XMLResource.OPTION_XML_MAP, xmlMap);
		return result;
	}

} //PmlUIExtensionModelResourceFactoryImpl
