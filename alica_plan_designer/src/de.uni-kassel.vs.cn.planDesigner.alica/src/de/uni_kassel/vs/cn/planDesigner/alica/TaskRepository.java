/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.common.util.EList;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Task Repository</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.TaskRepository#getTasks <em>Tasks</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.TaskRepository#getDefaultTask <em>Default Task</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getTaskRepository()
 * @model
 * @generated
 */
public interface TaskRepository extends PlanElement {
	/**
	 * Returns the value of the '<em><b>Tasks</b></em>' containment reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.Task}.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Tasks</em>' containment reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Tasks</em>' containment reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getTaskRepository_Tasks()
	 * @model containment="true"
	 * @generated
	 */
	EList<Task> getTasks();

	/**
	 * Returns the value of the '<em><b>Default Task</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Default Task</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Default Task</em>' reference.
	 * @see #setDefaultTask(Task)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getTaskRepository_DefaultTask()
	 * @model
	 * @generated
	 */
	Task getDefaultTask();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.TaskRepository#getDefaultTask <em>Default Task</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Default Task</em>' reference.
	 * @see #getDefaultTask()
	 * @generated
	 */
	void setDefaultTask(Task value);

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @model
	 * @generated
	 */
	Task createDefaultTask();

} // TaskRepository
