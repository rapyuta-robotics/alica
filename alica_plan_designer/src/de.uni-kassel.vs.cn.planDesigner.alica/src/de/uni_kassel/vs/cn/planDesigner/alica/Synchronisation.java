/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica;

import org.eclipse.emf.common.util.EList;

/**
 * <!-- begin-user-doc -->
 * A representation of the model object '<em><b>Synchronisation</b></em>'.
 * <!-- end-user-doc -->
 *
 * <p>
 * The following features are supported:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation#getSynchedTransitions <em>Synched Transitions</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation#getTalkTimeout <em>Talk Timeout</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation#getSyncTimeout <em>Sync Timeout</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation#isFailOnSyncTimeOut <em>Fail On Sync Time Out</em>}</li>
 * </ul>
 * </p>
 *
 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getSynchronisation()
 * @model
 * @generated
 */
public interface Synchronisation extends PlanElement {
	/**
	 * Returns the value of the '<em><b>Synched Transitions</b></em>' reference list.
	 * The list contents are of type {@link de.uni_kassel.vs.cn.planDesigner.alica.Transition}.
	 * It is bidirectional and its opposite is '{@link de.uni_kassel.vs.cn.planDesigner.alica.Transition#getSynchronisation <em>Synchronisation</em>}'.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Synched Transitions</em>' reference list isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Synched Transitions</em>' reference list.
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getSynchronisation_SynchedTransitions()
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.Transition#getSynchronisation
	 * @model opposite="synchronisation"
	 * @generated
	 */
	EList<Transition> getSynchedTransitions();

	/**
	 * Returns the value of the '<em><b>Talk Timeout</b></em>' attribute.
	 * The default value is <code>"30"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Talk Timeout</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Talk Timeout</em>' attribute.
	 * @see #setTalkTimeout(int)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getSynchronisation_TalkTimeout()
	 * @model default="30"
	 * @generated
	 */
	int getTalkTimeout();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation#getTalkTimeout <em>Talk Timeout</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Talk Timeout</em>' attribute.
	 * @see #getTalkTimeout()
	 * @generated
	 */
	void setTalkTimeout(int value);

	/**
	 * Returns the value of the '<em><b>Sync Timeout</b></em>' attribute.
	 * The default value is <code>"10000"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Sync Timeout</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Sync Timeout</em>' attribute.
	 * @see #setSyncTimeout(int)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getSynchronisation_SyncTimeout()
	 * @model default="10000"
	 * @generated
	 */
	int getSyncTimeout();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation#getSyncTimeout <em>Sync Timeout</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Sync Timeout</em>' attribute.
	 * @see #getSyncTimeout()
	 * @generated
	 */
	void setSyncTimeout(int value);

	/**
	 * Returns the value of the '<em><b>Fail On Sync Time Out</b></em>' attribute.
	 * The default value is <code>"false"</code>.
	 * <!-- begin-user-doc -->
	 * <p>
	 * If the meaning of the '<em>Fail On Sync Time Out</em>' attribute isn't clear,
	 * there really should be more of a description here...
	 * </p>
	 * <!-- end-user-doc -->
	 * @return the value of the '<em>Fail On Sync Time Out</em>' attribute.
	 * @see #setFailOnSyncTimeOut(boolean)
	 * @see de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage#getSynchronisation_FailOnSyncTimeOut()
	 * @model default="false"
	 * @generated
	 */
	boolean isFailOnSyncTimeOut();

	/**
	 * Sets the value of the '{@link de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation#isFailOnSyncTimeOut <em>Fail On Sync Time Out</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @param value the new value of the '<em>Fail On Sync Time Out</em>' attribute.
	 * @see #isFailOnSyncTimeOut()
	 * @generated
	 */
	void setFailOnSyncTimeOut(boolean value);

} // Synchronisation
