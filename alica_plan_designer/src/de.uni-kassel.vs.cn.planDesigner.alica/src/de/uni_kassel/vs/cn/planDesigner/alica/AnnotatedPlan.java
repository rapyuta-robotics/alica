/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;


/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Annotated Plan</b></em>'.
 * <!-- end-user-doc -->
 *
 * <!-- begin-model-doc -->
 * Wraps a plan, which is either activated or deactived in the associated plantype.
 * <!-- end-model-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan#getPlan <em>Plan</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan#isActivated <em>Activated</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAnnotatedPlan()
 * @model
 * @generated
 */
public interface AnnotatedPlan extends PlanElement {
	/**
	 * Returns the value of the '<em><b>Plan</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Plan</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Plan</em>' reference.
	 * @see #setPlan(Plan)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAnnotatedPlan_Plan()
	 * @model
	 * @generated
	 */
	Plan getPlan();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan#getPlan <em>Plan</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Plan</em>' reference.
	 * @see #getPlan()
	 * @generated
	 */
	void setPlan(Plan value);

	/**
	 * Returns the value of the '<em><b>Activated</b></em>' attribute.
	 * The default value is <code>"true"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Activated</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Activated</em>' attribute.
	 * @see #setActivated(boolean)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAnnotatedPlan_Activated()
	 * @model default="true"
	 * @generated
	 */
	boolean isActivated();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan#isActivated <em>Activated</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Activated</em>' attribute.
	 * @see #isActivated()
	 * @generated
	 */
	void setActivated(boolean value);

} // AnnotatedPlan
