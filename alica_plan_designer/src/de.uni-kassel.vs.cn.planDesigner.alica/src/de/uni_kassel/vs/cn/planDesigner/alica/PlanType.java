/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.common.util.EList;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Plan Type</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanType#getParametrisation <em>Parametrisation</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanType#getPlans <em>Plans</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanType()
 * @model
 * @generated
 */
public interface PlanType extends AbstractPlan {
	/**
	 * Returns the value of the '<em><b>Parametrisation</b></em>' containment reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation}.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Parametrisation</em>' containment reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Parametrisation</em>' containment reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanType_Parametrisation()
	 * @model containment="true"
	 * @generated
	 */
	EList<Parametrisation> getParametrisation();

	/**
	 * Returns the value of the '<em><b>Plans</b></em>' containment reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan}.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Plans</em>' containment reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Plans</em>' containment reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanType_Plans()
	 * @model containment="true"
	 * @generated
	 */
	EList<AnnotatedPlan> getPlans();

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @model
	 * @generated
	 */
	void ensureParametrisationConsistency();

} // PlanType
