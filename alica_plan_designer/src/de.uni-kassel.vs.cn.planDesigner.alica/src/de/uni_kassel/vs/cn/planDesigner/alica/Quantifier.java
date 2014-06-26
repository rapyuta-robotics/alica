/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.common.util.EList;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Quantifier</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Quantifier#getScope <em>Scope</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Quantifier#getSorts <em>Sorts</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getQuantifier()
 * @model abstract="true"
 * @generated
 */
public interface Quantifier extends PlanElement {
	/**
	 * Returns the value of the '<em><b>Scope</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Scope</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Scope</em>' reference.
	 * @see #setScope(IInhabitable)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getQuantifier_Scope()
	 * @model
	 * @generated
	 */
	IInhabitable getScope();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Quantifier#getScope <em>Scope</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Scope</em>' reference.
	 * @see #getScope()
	 * @generated
	 */
	void setScope(IInhabitable value);

	/**
	 * Returns the value of the '<em><b>Sorts</b></em>' attribute list.
	 * The list contents are of type {@link java.lang.String}.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Sorts</em>' attribute list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Sorts</em>' attribute list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getQuantifier_Sorts()
	 * @model default=""
	 * @generated
	 */
	EList<String> getSorts();

} // Quantifier
