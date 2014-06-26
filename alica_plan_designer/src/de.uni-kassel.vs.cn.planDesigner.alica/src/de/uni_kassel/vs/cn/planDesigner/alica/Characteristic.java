/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;


/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Characteristic</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Characteristic#getWeight <em>Weight</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Characteristic#getCapability <em>Capability</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Characteristic#getValue <em>Value</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getCharacteristic()
 * @model
 * @generated
 */
public interface Characteristic extends PlanElement {
	/**
	 * Returns the value of the '<em><b>Weight</b></em>' attribute.
	 * The default value is <code>"0"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Weight</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Weight</em>' attribute.
	 * @see #setWeight(double)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getCharacteristic_Weight()
	 * @model default="0"
	 * @generated
	 */
	double getWeight();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Characteristic#getWeight <em>Weight</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Weight</em>' attribute.
	 * @see #getWeight()
	 * @generated
	 */
	void setWeight(double value);

	/**
	 * Returns the value of the '<em><b>Capability</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Capability</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Capability</em>' reference.
	 * @see #setCapability(Capability)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getCharacteristic_Capability()
	 * @model
	 * @generated
	 */
	Capability getCapability();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Characteristic#getCapability <em>Capability</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Capability</em>' reference.
	 * @see #getCapability()
	 * @generated
	 */
	void setCapability(Capability value);

	/**
	 * Returns the value of the '<em><b>Value</b></em>' reference.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Value</em>' reference isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Value</em>' reference.
	 * @see #setValue(CapValue)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getCharacteristic_Value()
	 * @model
	 * @generated
	 */
	CapValue getValue();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Characteristic#getValue <em>Value</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Value</em>' reference.
	 * @see #getValue()
	 * @generated
	 */
	void setValue(CapValue value);

} // Characteristic
