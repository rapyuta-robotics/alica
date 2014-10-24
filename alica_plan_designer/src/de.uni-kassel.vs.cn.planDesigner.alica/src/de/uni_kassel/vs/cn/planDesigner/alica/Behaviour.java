/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.common.util.EList;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Behaviour</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Behaviour#getConfigurations <em>Configurations</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Behaviour#getDestinationPath <em>Destination Path</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getBehaviour()
 * @model
 * @generated
 */
public interface Behaviour extends PlanElement {
	/**
	 * Returns the value of the '<em><b>Configurations</b></em>' containment reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration}.
	 * It is bidirectional and its opposite is '{@link de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration#getBehaviour <em>Behaviour</em>}'.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Configurations</em>' containment reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Configurations</em>' containment reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getBehaviour_Configurations()
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration#getBehaviour
	 * @model opposite="behaviour" containment="true" required="true"
	 * @generated
	 */
	EList<BehaviourConfiguration> getConfigurations();

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
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getBehaviour_DestinationPath()
	 * @model default=""
	 * @generated
	 */
	String getDestinationPath();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Behaviour#getDestinationPath <em>Destination Path</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Destination Path</em>' attribute.
	 * @see #getDestinationPath()
	 * @generated
	 */
	void setDestinationPath(String value);

} // Behaviour
