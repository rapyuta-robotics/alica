/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Node</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Node#getInEdge <em>In Edge</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Node#getOutEdge <em>Out Edge</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getNode()
 * @model
 * @generated
 */
public interface Node extends EObject {
	/**
	 * Returns the value of the '<em><b>In Edge</b></em>' reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.Edge}.
	 * It is bidirectional and its opposite is '{@link de.uni_kassel.vs.cn.planDesigner.alica.Edge#getTo <em>To</em>}'.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>In Edge</em>' reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>In Edge</em>' reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getNode_InEdge()
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Edge#getTo
	 * @model opposite="to"
	 * @generated
	 */
	EList<Edge> getInEdge();

	/**
	 * Returns the value of the '<em><b>Out Edge</b></em>' reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.Edge}.
	 * It is bidirectional and its opposite is '{@link de.uni_kassel.vs.cn.planDesigner.alica.Edge#getFrom <em>From</em>}'.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Out Edge</em>' reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Out Edge</em>' reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getNode_OutEdge()
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Edge#getFrom
	 * @model opposite="from"
	 * @generated
	 */
	EList<Edge> getOutEdge();

} // Node
