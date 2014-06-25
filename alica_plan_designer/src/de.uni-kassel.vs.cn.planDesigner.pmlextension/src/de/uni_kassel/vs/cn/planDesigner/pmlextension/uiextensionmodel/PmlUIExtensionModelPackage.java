/**
 */
package de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel;

import org.eclipse.emf.ecore.EAttribute;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.EPackage;
import org.eclipse.emf.ecore.EReference;

/**
 * <!-- begin-user-doc -->
 * The <b>Package</b> for the model.
 * It contains accessors for the meta objects to represent
 * <ul>
 *   <li>each class,</li>
 *   <li>each feature of each class,</li>
 *   <li>each enum,</li>
 *   <li>and each data type</li>
 * </ul>
 * <!-- end-user-doc -->
 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelFactory
 * @model kind="package"
 *        annotation="http:///org/eclipse/emf/mapping/xsd2ecore/XSD2Ecore representation='schema' targetNamespace='http:///de.uni_kassel.vs.cn/alica/ui/extensionmodel'"
 * @generated
 */
public interface PmlUIExtensionModelPackage extends EPackage {
	/**
	 * The package name.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	String eNAME = "uiextensionmodel";

	/**
	 * The package namespace URI.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	String eNS_URI = "http:///de.uni_kassel.vs.cn/planDesigner/pmlextension/uiextensionmodel";

	/**
	 * The package namespace name.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	String eNS_PREFIX = "uiextensionmodel";

	/**
	 * The singleton instance of the package.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	PmlUIExtensionModelPackage eINSTANCE = de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl.init();

	/**
	 * The meta object id for the '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUiExtensionImpl <em>Pml Ui Extension</em>}' class.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUiExtensionImpl
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl#getPmlUiExtension()
	 * @generated
	 */
	int PML_UI_EXTENSION = 0;

	/**
	 * The feature id for the '<em><b>XPos</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int PML_UI_EXTENSION__XPOS = 0;

	/**
	 * The feature id for the '<em><b>YPos</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int PML_UI_EXTENSION__YPOS = 1;

	/**
	 * The feature id for the '<em><b>Width</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int PML_UI_EXTENSION__WIDTH = 2;

	/**
	 * The feature id for the '<em><b>Height</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int PML_UI_EXTENSION__HEIGHT = 3;

	/**
	 * The feature id for the '<em><b>Collapsed</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int PML_UI_EXTENSION__COLLAPSED = 4;

	/**
	 * The feature id for the '<em><b>Bendpoints</b></em>' containment reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int PML_UI_EXTENSION__BENDPOINTS = 5;

	/**
	 * The feature id for the '<em><b>Visible</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int PML_UI_EXTENSION__VISIBLE = 6;

	/**
	 * The number of structural features of the '<em>Pml Ui Extension</em>' class.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int PML_UI_EXTENSION_FEATURE_COUNT = 7;

	/**
	 * The meta object id for the '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUiExtensionMapImpl <em>Pml Ui Extension Map</em>}' class.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUiExtensionMapImpl
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl#getPmlUiExtensionMap()
	 * @generated
	 */
	int PML_UI_EXTENSION_MAP = 1;

	/**
	 * The feature id for the '<em><b>Extension</b></em>' map.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int PML_UI_EXTENSION_MAP__EXTENSION = 0;

	/**
	 * The number of structural features of the '<em>Pml Ui Extension Map</em>' class.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int PML_UI_EXTENSION_MAP_FEATURE_COUNT = 1;

	/**
	 * The meta object id for the '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.EObjectToPmlUiExtensionMapEntryImpl <em>EObject To Pml Ui Extension Map Entry</em>}' class.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.EObjectToPmlUiExtensionMapEntryImpl
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl#getEObjectToPmlUiExtensionMapEntry()
	 * @generated
	 */
	int EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY = 2;

	/**
	 * The feature id for the '<em><b>Key</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__KEY = 0;

	/**
	 * The feature id for the '<em><b>Value</b></em>' containment reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE = 1;

	/**
	 * The number of structural features of the '<em>EObject To Pml Ui Extension Map Entry</em>' class.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY_FEATURE_COUNT = 2;

	/**
	 * The meta object id for the '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.BendpointImpl <em>Bendpoint</em>}' class.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.BendpointImpl
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl#getBendpoint()
	 * @generated
	 */
	int BENDPOINT = 3;

	/**
	 * The feature id for the '<em><b>XPos</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int BENDPOINT__XPOS = 0;

	/**
	 * The feature id for the '<em><b>YPos</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int BENDPOINT__YPOS = 1;

	/**
	 * The number of structural features of the '<em>Bendpoint</em>' class.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 * @ordered
	 */
	int BENDPOINT_FEATURE_COUNT = 2;


	/**
	 * Returns the meta object for class '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension <em>Pml Ui Extension</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for class '<em>Pml Ui Extension</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension
	 * @generated
	 */
	EClass getPmlUiExtension();

	/**
	 * Returns the meta object for the attribute '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#getXPos <em>XPos</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the attribute '<em>XPos</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#getXPos()
	 * @see #getPmlUiExtension()
	 * @generated
	 */
	EAttribute getPmlUiExtension_XPos();

	/**
	 * Returns the meta object for the attribute '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#getYPos <em>YPos</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the attribute '<em>YPos</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#getYPos()
	 * @see #getPmlUiExtension()
	 * @generated
	 */
	EAttribute getPmlUiExtension_YPos();

	/**
	 * Returns the meta object for the attribute '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#getWidth <em>Width</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the attribute '<em>Width</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#getWidth()
	 * @see #getPmlUiExtension()
	 * @generated
	 */
	EAttribute getPmlUiExtension_Width();

	/**
	 * Returns the meta object for the attribute '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#getHeight <em>Height</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the attribute '<em>Height</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#getHeight()
	 * @see #getPmlUiExtension()
	 * @generated
	 */
	EAttribute getPmlUiExtension_Height();

	/**
	 * Returns the meta object for the attribute '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#isCollapsed <em>Collapsed</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the attribute '<em>Collapsed</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#isCollapsed()
	 * @see #getPmlUiExtension()
	 * @generated
	 */
	EAttribute getPmlUiExtension_Collapsed();

	/**
	 * Returns the meta object for the containment reference list '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#getBendpoints <em>Bendpoints</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the containment reference list '<em>Bendpoints</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#getBendpoints()
	 * @see #getPmlUiExtension()
	 * @generated
	 */
	EReference getPmlUiExtension_Bendpoints();

	/**
	 * Returns the meta object for the attribute '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#isVisible <em>Visible</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the attribute '<em>Visible</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension#isVisible()
	 * @see #getPmlUiExtension()
	 * @generated
	 */
	EAttribute getPmlUiExtension_Visible();

	/**
	 * Returns the meta object for class '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap <em>Pml Ui Extension Map</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for class '<em>Pml Ui Extension Map</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap
	 * @generated
	 */
	EClass getPmlUiExtensionMap();

	/**
	 * Returns the meta object for the map '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap#getExtension <em>Extension</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the map '<em>Extension</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap#getExtension()
	 * @see #getPmlUiExtensionMap()
	 * @generated
	 */
	EReference getPmlUiExtensionMap_Extension();

	/**
	 * Returns the meta object for class '{@link java.util.Map.Entry <em>EObject To Pml Ui Extension Map Entry</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for class '<em>EObject To Pml Ui Extension Map Entry</em>'.
	 * @see java.util.Map.Entry
	 * @model keyType="org.eclipse.emf.ecore.EObject"
	 *        valueType="de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension" valueContainment="true"
	 * @generated
	 */
	EClass getEObjectToPmlUiExtensionMapEntry();

	/**
	 * Returns the meta object for the reference '{@link java.util.Map.Entry <em>Key</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the reference '<em>Key</em>'.
	 * @see java.util.Map.Entry
	 * @see #getEObjectToPmlUiExtensionMapEntry()
	 * @generated
	 */
	EReference getEObjectToPmlUiExtensionMapEntry_Key();

	/**
	 * Returns the meta object for the containment reference '{@link java.util.Map.Entry <em>Value</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the containment reference '<em>Value</em>'.
	 * @see java.util.Map.Entry
	 * @see #getEObjectToPmlUiExtensionMapEntry()
	 * @generated
	 */
	EReference getEObjectToPmlUiExtensionMapEntry_Value();

	/**
	 * Returns the meta object for class '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.Bendpoint <em>Bendpoint</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for class '<em>Bendpoint</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.Bendpoint
	 * @generated
	 */
	EClass getBendpoint();

	/**
	 * Returns the meta object for the attribute '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.Bendpoint#getXPos <em>XPos</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the attribute '<em>XPos</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.Bendpoint#getXPos()
	 * @see #getBendpoint()
	 * @generated
	 */
	EAttribute getBendpoint_XPos();

	/**
	 * Returns the meta object for the attribute '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.Bendpoint#getYPos <em>YPos</em>}'.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the meta object for the attribute '<em>YPos</em>'.
	 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.Bendpoint#getYPos()
	 * @see #getBendpoint()
	 * @generated
	 */
	EAttribute getBendpoint_YPos();

	/**
	 * Returns the factory that creates the instances of the model.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @return the factory that creates the instances of the model.
	 * @generated
	 */
	PmlUIExtensionModelFactory getPmlUIExtensionModelFactory();

	/**
	 * <!-- begin-user-doc -->
	 * Defines literals for the meta objects that represent
	 * <ul>
	 *   <li>each class,</li>
	 *   <li>each feature of each class,</li>
	 *   <li>each enum,</li>
	 *   <li>and each data type</li>
	 * </ul>
	 * <!-- end-user-doc -->
	 * @generated
	 */
	interface Literals {
		/**
		 * The meta object literal for the '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUiExtensionImpl <em>Pml Ui Extension</em>}' class.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUiExtensionImpl
		 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl#getPmlUiExtension()
		 * @generated
		 */
		EClass PML_UI_EXTENSION = eINSTANCE.getPmlUiExtension();

		/**
		 * The meta object literal for the '<em><b>XPos</b></em>' attribute feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EAttribute PML_UI_EXTENSION__XPOS = eINSTANCE.getPmlUiExtension_XPos();

		/**
		 * The meta object literal for the '<em><b>YPos</b></em>' attribute feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EAttribute PML_UI_EXTENSION__YPOS = eINSTANCE.getPmlUiExtension_YPos();

		/**
		 * The meta object literal for the '<em><b>Width</b></em>' attribute feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EAttribute PML_UI_EXTENSION__WIDTH = eINSTANCE.getPmlUiExtension_Width();

		/**
		 * The meta object literal for the '<em><b>Height</b></em>' attribute feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EAttribute PML_UI_EXTENSION__HEIGHT = eINSTANCE.getPmlUiExtension_Height();

		/**
		 * The meta object literal for the '<em><b>Collapsed</b></em>' attribute feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EAttribute PML_UI_EXTENSION__COLLAPSED = eINSTANCE.getPmlUiExtension_Collapsed();

		/**
		 * The meta object literal for the '<em><b>Bendpoints</b></em>' containment reference list feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EReference PML_UI_EXTENSION__BENDPOINTS = eINSTANCE.getPmlUiExtension_Bendpoints();

		/**
		 * The meta object literal for the '<em><b>Visible</b></em>' attribute feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EAttribute PML_UI_EXTENSION__VISIBLE = eINSTANCE.getPmlUiExtension_Visible();

		/**
		 * The meta object literal for the '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUiExtensionMapImpl <em>Pml Ui Extension Map</em>}' class.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUiExtensionMapImpl
		 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl#getPmlUiExtensionMap()
		 * @generated
		 */
		EClass PML_UI_EXTENSION_MAP = eINSTANCE.getPmlUiExtensionMap();

		/**
		 * The meta object literal for the '<em><b>Extension</b></em>' map feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EReference PML_UI_EXTENSION_MAP__EXTENSION = eINSTANCE.getPmlUiExtensionMap_Extension();

		/**
		 * The meta object literal for the '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.EObjectToPmlUiExtensionMapEntryImpl <em>EObject To Pml Ui Extension Map Entry</em>}' class.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.EObjectToPmlUiExtensionMapEntryImpl
		 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl#getEObjectToPmlUiExtensionMapEntry()
		 * @generated
		 */
		EClass EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY = eINSTANCE.getEObjectToPmlUiExtensionMapEntry();

		/**
		 * The meta object literal for the '<em><b>Key</b></em>' reference feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EReference EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__KEY = eINSTANCE.getEObjectToPmlUiExtensionMapEntry_Key();

		/**
		 * The meta object literal for the '<em><b>Value</b></em>' containment reference feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EReference EOBJECT_TO_PML_UI_EXTENSION_MAP_ENTRY__VALUE = eINSTANCE.getEObjectToPmlUiExtensionMapEntry_Value();

		/**
		 * The meta object literal for the '{@link de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.BendpointImpl <em>Bendpoint</em>}' class.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.BendpointImpl
		 * @see de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl#getBendpoint()
		 * @generated
		 */
		EClass BENDPOINT = eINSTANCE.getBendpoint();

		/**
		 * The meta object literal for the '<em><b>XPos</b></em>' attribute feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EAttribute BENDPOINT__XPOS = eINSTANCE.getBendpoint_XPos();

		/**
		 * The meta object literal for the '<em><b>YPos</b></em>' attribute feature.
		 * <!-- begin-user-doc -->
		 * <!-- end-user-doc -->
		 * @generated
		 */
		EAttribute BENDPOINT__YPOS = eINSTANCE.getBendpoint_YPos();

	}

} //PmlUIExtensionModelPackage
