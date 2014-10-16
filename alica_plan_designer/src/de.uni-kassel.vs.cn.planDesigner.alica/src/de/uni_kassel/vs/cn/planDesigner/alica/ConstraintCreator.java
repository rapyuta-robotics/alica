/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.common.util.EList;

import org.eclipse.emf.ecore.EObject;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Constraint Creator</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.ConstraintCreator#getConditions <em>Conditions</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.ConstraintCreator#getPlans <em>Plans</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getConstraintCreator()
 * @model
 * @generated
 */
public interface ConstraintCreator extends EObject {
	/**
	 * Returns the value of the '<em><b>Conditions</b></em>' reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.Condition}.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Conditions</em>' reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Conditions</em>' reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getConstraintCreator_Conditions()
	 * @model ordered="false"
	 * @generated
	 */
	EList<Condition> getConditions();

	/**
	 * Returns the value of the '<em><b>Plans</b></em>' reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan}.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Plans</em>' reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Plans</em>' reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getConstraintCreator_Plans()
	 * @model
	 * @generated
	 */
	EList<AbstractPlan> getPlans();

} // ConstraintCreator
