/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;


/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Parametrisation</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation#getSubplan <em>Subplan</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation#getSubvar <em>Subvar</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation#getVar <em>Var</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getParametrisation()
 * @model
 * @generated
 */
public interface Parametrisation extends PlanElement {
	/**
	 * Returns the value of the '<em><b>Subplan</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Subplan</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Subplan</em>' reference.
	 * @see #setSubplan(AbstractPlan)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getParametrisation_Subplan()
	 * @model required="true"
	 * @generated
	 */
	AbstractPlan getSubplan();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation#getSubplan <em>Subplan</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Subplan</em>' reference.
	 * @see #getSubplan()
	 * @generated
	 */
	void setSubplan(AbstractPlan value);

	/**
	 * Returns the value of the '<em><b>Subvar</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Subvar</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Subvar</em>' reference.
	 * @see #setSubvar(Variable)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getParametrisation_Subvar()
	 * @model required="true"
	 * @generated
	 */
	Variable getSubvar();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation#getSubvar <em>Subvar</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Subvar</em>' reference.
	 * @see #getSubvar()
	 * @generated
	 */
	void setSubvar(Variable value);

	/**
	 * Returns the value of the '<em><b>Var</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Var</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Var</em>' reference.
	 * @see #setVar(Variable)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getParametrisation_Var()
	 * @model
	 * @generated
	 */
	Variable getVar();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation#getVar <em>Var</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Var</em>' reference.
	 * @see #getVar()
	 * @generated
	 */
	void setVar(Variable value);

} // Parametrisation
