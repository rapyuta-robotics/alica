/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;


/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Entry Point</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#getTask <em>Task</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#isSuccessRequired <em>Success Required</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#getState <em>State</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#getMinCardinality <em>Min Cardinality</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#getMaxCardinality <em>Max Cardinality</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#getPlan <em>Plan</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getEntryPoint()
 * @model
 * @generated
 */
public interface EntryPoint extends IInhabitable {
	/**
	 * Returns the value of the '<em><b>Task</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Task</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Task</em>' reference.
	 * @see #setTask(Task)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getEntryPoint_Task()
	 * @model required="true"
	 * @generated
	 */
	Task getTask();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#getTask <em>Task</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Task</em>' reference.
	 * @see #getTask()
	 * @generated
	 */
	void setTask(Task value);

	/**
	 * Returns the value of the '<em><b>Success Required</b></em>' attribute.
	 * The default value is <code>"false"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Success Required</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Success Required</em>' attribute.
	 * @see #setSuccessRequired(boolean)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getEntryPoint_SuccessRequired()
	 * @model default="false"
	 * @generated
	 */
	boolean isSuccessRequired();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#isSuccessRequired <em>Success Required</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Success Required</em>' attribute.
	 * @see #isSuccessRequired()
	 * @generated
	 */
	void setSuccessRequired(boolean value);

	/**
	 * Returns the value of the '<em><b>State</b></em>' reference.
	 * It is bidirectional and its opposite is '{@link de.uni_kassel.vs.cn.planDesigner.alica.State#getEntryPoint <em>Entry Point</em>}'.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>State</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>State</em>' reference.
	 * @see #setState(State)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getEntryPoint_State()
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.State#getEntryPoint
	 * @model opposite="entryPoint"
	 * @generated
	 */
	State getState();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#getState <em>State</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>State</em>' reference.
	 * @see #getState()
	 * @generated
	 */
	void setState(State value);

	/**
	 * Returns the value of the '<em><b>Min Cardinality</b></em>' attribute.
	 * The default value is <code>"0"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Min Cardinality</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Min Cardinality</em>' attribute.
	 * @see #setMinCardinality(int)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getEntryPoint_MinCardinality()
	 * @model default="0" required="true"
	 * @generated
	 */
	int getMinCardinality();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#getMinCardinality <em>Min Cardinality</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Min Cardinality</em>' attribute.
	 * @see #getMinCardinality()
	 * @generated
	 */
	void setMinCardinality(int value);

	/**
	 * Returns the value of the '<em><b>Max Cardinality</b></em>' attribute.
	 * The default value is <code>"2147483647"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Max Cardinality</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Max Cardinality</em>' attribute.
	 * @see #setMaxCardinality(int)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getEntryPoint_MaxCardinality()
	 * @model default="2147483647" required="true"
	 * @generated
	 */
	int getMaxCardinality();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#getMaxCardinality <em>Max Cardinality</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Max Cardinality</em>' attribute.
	 * @see #getMaxCardinality()
	 * @generated
	 */
	void setMaxCardinality(int value);

	/**
	 * Returns the value of the '<em><b>Plan</b></em>' container reference.
	 * It is bidirectional and its opposite is '{@link de.uni_kassel.vs.cn.planDesigner.alica.Plan#getEntryPoints <em>Entry Points</em>}'.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Plan</em>' container reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Plan</em>' container reference.
	 * @see #setPlan(Plan)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getEntryPoint_Plan()
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Plan#getEntryPoints
	 * @model opposite="entryPoints" required="true" transient="false"
	 * @generated
	 */
	Plan getPlan();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint#getPlan <em>Plan</em>}' container reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Plan</em>' container reference.
	 * @see #getPlan()
	 * @generated
	 */
	void setPlan(Plan value);

} // EntryPoint
