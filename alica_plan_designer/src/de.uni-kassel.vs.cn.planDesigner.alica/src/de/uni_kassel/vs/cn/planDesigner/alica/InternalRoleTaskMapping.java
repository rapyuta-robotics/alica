/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.ecore.EObject;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Internal Role Task Mapping</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping#getRole <em>Role</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping#getPriority <em>Priority</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getInternalRoleTaskMapping()
 * @model
 * @generated
 */
public interface InternalRoleTaskMapping extends EObject {
	/**
	 * Returns the value of the '<em><b>Role</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Role</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Role</em>' reference.
	 * @see #setRole(Role)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getInternalRoleTaskMapping_Role()
	 * @model
	 * @generated
	 */
	Role getRole();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping#getRole <em>Role</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Role</em>' reference.
	 * @see #getRole()
	 * @generated
	 */
	void setRole(Role value);

	/**
	 * Returns the value of the '<em><b>Priority</b></em>' attribute.
	 * The default value is <code>"0.0"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Priority</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Priority</em>' attribute.
	 * @see #setPriority(double)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getInternalRoleTaskMapping_Priority()
	 * @model default="0.0"
	 * @generated
	 */
	double getPriority();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping#getPriority <em>Priority</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Priority</em>' attribute.
	 * @see #getPriority()
	 * @generated
	 */
	void setPriority(double value);

} // InternalRoleTaskMapping
