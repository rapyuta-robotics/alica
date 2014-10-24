/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.common.util.EList;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Abstract Plan</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#getRating <em>Rating</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#getConditions <em>Conditions</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#isMasterPlan <em>Master Plan</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#getUtilityFunction <em>Utility Function</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#getUtilityThreshold <em>Utility Threshold</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#getVars <em>Vars</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#getDestinationPath <em>Destination Path</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAbstractPlan()
 * @model abstract="true"
 * @generated
 */
public interface AbstractPlan extends PlanElement, IInhabitable {
	/**
	 * Returns the value of the '<em><b>Rating</b></em>' containment reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Rating</em>' containment reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Rating</em>' containment reference.
	 * @see #setRating(Rating)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAbstractPlan_Rating()
	 * @model containment="true"
	 * @generated
	 */
	Rating getRating();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#getRating <em>Rating</em>}' containment reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Rating</em>' containment reference.
	 * @see #getRating()
	 * @generated
	 */
	void setRating(Rating value);

	/**
	 * Returns the value of the '<em><b>Conditions</b></em>' containment reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.Condition}.
	 * It is bidirectional and its opposite is '{@link de.uni_kassel.vs.cn.planDesigner.alica.Condition#getAbstractPlan <em>Abstract Plan</em>}'.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Conditions</em>' containment reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Conditions</em>' containment reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAbstractPlan_Conditions()
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Condition#getAbstractPlan
	 * @model opposite="abstractPlan" containment="true"
	 * @generated
	 */
	EList<Condition> getConditions();

	/**
	 * Returns the value of the '<em><b>Master Plan</b></em>' attribute.
	 * The default value is <code>"false"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Master Plan</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Master Plan</em>' attribute.
	 * @see #setMasterPlan(boolean)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAbstractPlan_MasterPlan()
	 * @model default="false"
	 * @generated
	 */
	boolean isMasterPlan();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#isMasterPlan <em>Master Plan</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Master Plan</em>' attribute.
	 * @see #isMasterPlan()
	 * @generated
	 */
	void setMasterPlan(boolean value);

	/**
	 * Returns the value of the '<em><b>Utility Function</b></em>' attribute.
	 * The default value is <code>""</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Utility Function</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Utility Function</em>' attribute.
	 * @see #setUtilityFunction(String)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAbstractPlan_UtilityFunction()
	 * @model default=""
	 * @generated
	 */
	String getUtilityFunction();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#getUtilityFunction <em>Utility Function</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Utility Function</em>' attribute.
	 * @see #getUtilityFunction()
	 * @generated
	 */
	void setUtilityFunction(String value);

	/**
	 * Returns the value of the '<em><b>Utility Threshold</b></em>' attribute.
	 * The default value is <code>"0.1"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Utility Threshold</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Utility Threshold</em>' attribute.
	 * @see #setUtilityThreshold(double)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAbstractPlan_UtilityThreshold()
	 * @model default="0.1"
	 * @generated
	 */
	double getUtilityThreshold();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#getUtilityThreshold <em>Utility Threshold</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Utility Threshold</em>' attribute.
	 * @see #getUtilityThreshold()
	 * @generated
	 */
	void setUtilityThreshold(double value);

	/**
	 * Returns the value of the '<em><b>Vars</b></em>' containment reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.Variable}.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Vars</em>' containment reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Vars</em>' containment reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAbstractPlan_Vars()
	 * @model containment="true"
	 * @generated
	 */
	EList<Variable> getVars();

	/**
	 * Returns the value of the '<em><b>Destination Path</b></em>' attribute.
	 * The default value is <code>""</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Destination Path</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Destination Path</em>' attribute.
	 * @see #setDestinationPath(String)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getAbstractPlan_DestinationPath()
	 * @model default=""
	 * @generated
	 */
	String getDestinationPath();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan#getDestinationPath <em>Destination Path</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Destination Path</em>' attribute.
	 * @see #getDestinationPath()
	 * @generated
	 */
	void setDestinationPath(String value);

} // AbstractPlan
