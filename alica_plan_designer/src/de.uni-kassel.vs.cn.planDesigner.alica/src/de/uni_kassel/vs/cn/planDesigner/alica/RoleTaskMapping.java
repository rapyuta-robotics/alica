/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.common.util.EMap;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Role Task Mapping</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping#getTaskPriorities <em>Task Priorities</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping#getRole <em>Role</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getRoleTaskMapping()
 * @model
 * @generated
 */
public interface RoleTaskMapping extends PlanElement {
	/**
	 * Returns the value of the '<em><b>Task Priorities</b></em>' map.
	 * The key is of type {@link java.lang.Long},
	 * and the value is of type {@link java.lang.Double},
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Task Priorities</em>' map isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Task Priorities</em>' map.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getRoleTaskMapping_TaskPriorities()
	 * @model mapType="de.uni_kassel.vs.cn.planDesigner.alica.ELongToDoubleMapEntry<org.eclipse.emf.ecore.ELongObject, org.eclipse.emf.ecore.EDoubleObject>"
	 * @generated
	 */
	EMap<Long, Double> getTaskPriorities();

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
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getRoleTaskMapping_Role()
	 * @model
	 * @generated
	 */
	Role getRole();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping#getRole <em>Role</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Role</em>' reference.
	 * @see #getRole()
	 * @generated
	 */
	void setRole(Role value);

} // RoleTaskMapping
