/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.ecore.EObject;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Plan Element</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanElement#getId <em>Id</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanElement#getName <em>Name</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanElement#getComment <em>Comment</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanElement()
 * @model abstract="true"
 * @generated
 */
public interface PlanElement extends EObject {
	/**
	 * Returns the value of the '<em><b>Id</b></em>' attribute.
	 * The default value is <code>"0"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Id</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Id</em>' attribute.
	 * @see #setId(long)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanElement_Id()
	 * @model default="0" id="true" required="true"
	 * @generated
	 */
	long getId();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanElement#getId <em>Id</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Id</em>' attribute.
	 * @see #getId()
	 * @generated
	 */
	void setId(long value);

	/**
	 * Returns the value of the '<em><b>Name</b></em>' attribute.
	 * The default value is <code>"MISSING_NAME"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Name</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Name</em>' attribute.
	 * @see #setName(String)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanElement_Name()
	 * @model default="MISSING_NAME"
	 * @generated
	 */
	String getName();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanElement#getName <em>Name</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Name</em>' attribute.
	 * @see #getName()
	 * @generated
	 */
	void setName(String value);

	/**
	 * Returns the value of the '<em><b>Comment</b></em>' attribute.
	 * The default value is <code>""</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Comment</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Comment</em>' attribute.
	 * @see #setComment(String)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getPlanElement_Comment()
	 * @model default=""
	 * @generated
	 */
	String getComment();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.PlanElement#getComment <em>Comment</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Comment</em>' attribute.
	 * @see #getComment()
	 * @generated
	 */
	void setComment(String value);

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @model
	 * @generated
	 */
	long generateID();

} // PlanElement
