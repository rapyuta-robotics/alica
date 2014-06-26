/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.common.util.EList;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Planning Problem</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getPlans <em>Plans</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getPlanner <em>Planner</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getAlternativePlan <em>Alternative Plan</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getWaitPlan <em>Wait Plan</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getUpdateRate <em>Update Rate</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#isDistributeProblem <em>Distribute Problem</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getPlanningType <em>Planning Type</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getRequirements <em>Requirements</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getPlannerParams <em>Planner Params</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningProblem()
 * @model
 * @generated
 */
public interface PlanningProblem extends AbstractPlan {
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
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningProblem_Plans()
	 * @model ordered="false"
	 * @generated
	 */
	EList<AbstractPlan> getPlans();

	/**
	 * Returns the value of the '<em><b>Planner</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Planner</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Planner</em>' reference.
	 * @see #setPlanner(Planner)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningProblem_Planner()
	 * @model ordered="false"
	 * @generated
	 */
	Planner getPlanner();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getPlanner <em>Planner</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Planner</em>' reference.
	 * @see #getPlanner()
	 * @generated
	 */
	void setPlanner(Planner value);

	/**
	 * Returns the value of the '<em><b>Alternative Plan</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Alternative Plan</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Alternative Plan</em>' reference.
	 * @see #setAlternativePlan(AbstractPlan)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningProblem_AlternativePlan()
	 * @model
	 * @generated
	 */
	AbstractPlan getAlternativePlan();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getAlternativePlan <em>Alternative Plan</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Alternative Plan</em>' reference.
	 * @see #getAlternativePlan()
	 * @generated
	 */
	void setAlternativePlan(AbstractPlan value);

	/**
	 * Returns the value of the '<em><b>Wait Plan</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Wait Plan</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Wait Plan</em>' reference.
	 * @see #setWaitPlan(AbstractPlan)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningProblem_WaitPlan()
	 * @model
	 * @generated
	 */
	AbstractPlan getWaitPlan();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getWaitPlan <em>Wait Plan</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Wait Plan</em>' reference.
	 * @see #getWaitPlan()
	 * @generated
	 */
	void setWaitPlan(AbstractPlan value);

	/**
	 * Returns the value of the '<em><b>Update Rate</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Update Rate</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Update Rate</em>' attribute.
	 * @see #setUpdateRate(int)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningProblem_UpdateRate()
	 * @model
	 * @generated
	 */
	int getUpdateRate();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getUpdateRate <em>Update Rate</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Update Rate</em>' attribute.
	 * @see #getUpdateRate()
	 * @generated
	 */
	void setUpdateRate(int value);

	/**
	 * Returns the value of the '<em><b>Distribute Problem</b></em>' attribute.
	 * The default value is <code>"false"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Distribute Problem</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Distribute Problem</em>' attribute.
	 * @see #setDistributeProblem(boolean)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningProblem_DistributeProblem()
	 * @model default="false"
	 * @generated
	 */
	boolean isDistributeProblem();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#isDistributeProblem <em>Distribute Problem</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Distribute Problem</em>' attribute.
	 * @see #isDistributeProblem()
	 * @generated
	 */
	void setDistributeProblem(boolean value);

	/**
	 * Returns the value of the '<em><b>Planning Type</b></em>' attribute.
	 * The default value is <code>"Online"</code>.
	 * The literals are from the enumeration {@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningType}.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Planning Type</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Planning Type</em>' attribute.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.PlanningType
	 * @see #setPlanningType(PlanningType)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningProblem_PlanningType()
	 * @model default="Online" required="true"
	 * @generated
	 */
	PlanningType getPlanningType();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getPlanningType <em>Planning Type</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Planning Type</em>' attribute.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.PlanningType
	 * @see #getPlanningType()
	 * @generated
	 */
	void setPlanningType(PlanningType value);

	/**
	 * Returns the value of the '<em><b>Requirements</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Requirements</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Requirements</em>' attribute.
	 * @see #setRequirements(String)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningProblem_Requirements()
	 * @model
	 * @generated
	 */
	String getRequirements();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getRequirements <em>Requirements</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Requirements</em>' attribute.
	 * @see #getRequirements()
	 * @generated
	 */
	void setRequirements(String value);

	/**
	 * Returns the value of the '<em><b>Planner Params</b></em>' attribute.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Planner Params</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Planner Params</em>' attribute.
	 * @see #setPlannerParams(String)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanningProblem_PlannerParams()
	 * @model
	 * @generated
	 */
	String getPlannerParams();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem#getPlannerParams <em>Planner Params</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Planner Params</em>' attribute.
	 * @see #getPlannerParams()
	 * @generated
	 */
	void setPlannerParams(String value);

} // PlanningProblem
