/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica.impl;

import java.util.Collection;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.notify.NotificationChain;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.InternalEObject;
import org.eclipse.emf.ecore.impl.ENotificationImpl;
import org.eclipse.emf.ecore.util.EObjectWithInverseResolvingEList;
import org.eclipse.emf.ecore.util.InternalEList;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>Synchronisation</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.SynchronisationImpl#getSynchedTransitions <em>Synched Transitions</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.SynchronisationImpl#getTalkTimeout <em>Talk Timeout</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.SynchronisationImpl#getSyncTimeout <em>Sync Timeout</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.SynchronisationImpl#isFailOnSyncTimeOut <em>Fail On Sync Time Out</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class SynchronisationImpl extends PlanElementImpl implements Synchronisation {
	/**
	 * The cached value of the '{@link #getSynchedTransitions() <em>Synched Transitions</em>}' reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getSynchedTransitions()
	 * @generated
	 * @ordered
	 */
	protected EList<Transition> synchedTransitions;

	/**
	 * The default value of the '{@link #getTalkTimeout() <em>Talk Timeout</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getTalkTimeout()
	 * @generated
	 * @ordered
	 */
	protected static final int TALK_TIMEOUT_EDEFAULT = 30;

	/**
	 * The cached value of the '{@link #getTalkTimeout() <em>Talk Timeout</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getTalkTimeout()
	 * @generated
	 * @ordered
	 */
	protected int talkTimeout = TALK_TIMEOUT_EDEFAULT;

	/**
	 * The default value of the '{@link #getSyncTimeout() <em>Sync Timeout</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getSyncTimeout()
	 * @generated
	 * @ordered
	 */
	protected static final int SYNC_TIMEOUT_EDEFAULT = 10000;

	/**
	 * The cached value of the '{@link #getSyncTimeout() <em>Sync Timeout</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getSyncTimeout()
	 * @generated
	 * @ordered
	 */
	protected int syncTimeout = SYNC_TIMEOUT_EDEFAULT;

	/**
	 * The default value of the '{@link #isFailOnSyncTimeOut() <em>Fail On Sync Time Out</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #isFailOnSyncTimeOut()
	 * @generated
	 * @ordered
	 */
	protected static final boolean FAIL_ON_SYNC_TIME_OUT_EDEFAULT = false;

	/**
	 * The cached value of the '{@link #isFailOnSyncTimeOut() <em>Fail On Sync Time Out</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #isFailOnSyncTimeOut()
	 * @generated
	 * @ordered
	 */
	protected boolean failOnSyncTimeOut = FAIL_ON_SYNC_TIME_OUT_EDEFAULT;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected SynchronisationImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return AlicaPackage.Literals.SYNCHRONISATION;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<Transition> getSynchedTransitions() {
		if (synchedTransitions == null) {
			synchedTransitions = new EObjectWithInverseResolvingEList<Transition>(Transition.class, this, AlicaPackage.SYNCHRONISATION__SYNCHED_TRANSITIONS, AlicaPackage.TRANSITION__SYNCHRONISATION);
		}
		return synchedTransitions;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public int getTalkTimeout() {
		return talkTimeout;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setTalkTimeout(int newTalkTimeout) {
		int oldTalkTimeout = talkTimeout;
		talkTimeout = newTalkTimeout;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.SYNCHRONISATION__TALK_TIMEOUT, oldTalkTimeout, talkTimeout));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public int getSyncTimeout() {
		return syncTimeout;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setSyncTimeout(int newSyncTimeout) {
		int oldSyncTimeout = syncTimeout;
		syncTimeout = newSyncTimeout;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.SYNCHRONISATION__SYNC_TIMEOUT, oldSyncTimeout, syncTimeout));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public boolean isFailOnSyncTimeOut() {
		return failOnSyncTimeOut;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setFailOnSyncTimeOut(boolean newFailOnSyncTimeOut) {
		boolean oldFailOnSyncTimeOut = failOnSyncTimeOut;
		failOnSyncTimeOut = newFailOnSyncTimeOut;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.SYNCHRONISATION__FAIL_ON_SYNC_TIME_OUT, oldFailOnSyncTimeOut, failOnSyncTimeOut));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@SuppressWarnings("unchecked")
	@Override
	public NotificationChain eInverseAdd(InternalEObject otherEnd, int featureID, NotificationChain msgs) {
		switch (featureID) {
			case AlicaPackage.SYNCHRONISATION__SYNCHED_TRANSITIONS:
				return ((InternalEList<InternalEObject>)(InternalEList<?>)getSynchedTransitions()).basicAdd(otherEnd, msgs);
		}
		return super.eInverseAdd(otherEnd, featureID, msgs);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public NotificationChain eInverseRemove(InternalEObject otherEnd, int featureID, NotificationChain msgs) {
		switch (featureID) {
			case AlicaPackage.SYNCHRONISATION__SYNCHED_TRANSITIONS:
				return ((InternalEList<?>)getSynchedTransitions()).basicRemove(otherEnd, msgs);
		}
		return super.eInverseRemove(otherEnd, featureID, msgs);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public Object eGet(int featureID, boolean resolve, boolean coreType) {
		switch (featureID) {
			case AlicaPackage.SYNCHRONISATION__SYNCHED_TRANSITIONS:
				return getSynchedTransitions();
			case AlicaPackage.SYNCHRONISATION__TALK_TIMEOUT:
				return getTalkTimeout();
			case AlicaPackage.SYNCHRONISATION__SYNC_TIMEOUT:
				return getSyncTimeout();
			case AlicaPackage.SYNCHRONISATION__FAIL_ON_SYNC_TIME_OUT:
				return isFailOnSyncTimeOut();
		}
		return super.eGet(featureID, resolve, coreType);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@SuppressWarnings("unchecked")
	@Override
	public void eSet(int featureID, Object newValue) {
		switch (featureID) {
			case AlicaPackage.SYNCHRONISATION__SYNCHED_TRANSITIONS:
				getSynchedTransitions().clear();
				getSynchedTransitions().addAll((Collection<? extends Transition>)newValue);
				return;
			case AlicaPackage.SYNCHRONISATION__TALK_TIMEOUT:
				setTalkTimeout((Integer)newValue);
				return;
			case AlicaPackage.SYNCHRONISATION__SYNC_TIMEOUT:
				setSyncTimeout((Integer)newValue);
				return;
			case AlicaPackage.SYNCHRONISATION__FAIL_ON_SYNC_TIME_OUT:
				setFailOnSyncTimeOut((Boolean)newValue);
				return;
		}
		super.eSet(featureID, newValue);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public void eUnset(int featureID) {
		switch (featureID) {
			case AlicaPackage.SYNCHRONISATION__SYNCHED_TRANSITIONS:
				getSynchedTransitions().clear();
				return;
			case AlicaPackage.SYNCHRONISATION__TALK_TIMEOUT:
				setTalkTimeout(TALK_TIMEOUT_EDEFAULT);
				return;
			case AlicaPackage.SYNCHRONISATION__SYNC_TIMEOUT:
				setSyncTimeout(SYNC_TIMEOUT_EDEFAULT);
				return;
			case AlicaPackage.SYNCHRONISATION__FAIL_ON_SYNC_TIME_OUT:
				setFailOnSyncTimeOut(FAIL_ON_SYNC_TIME_OUT_EDEFAULT);
				return;
		}
		super.eUnset(featureID);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public boolean eIsSet(int featureID) {
		switch (featureID) {
			case AlicaPackage.SYNCHRONISATION__SYNCHED_TRANSITIONS:
				return synchedTransitions != null && !synchedTransitions.isEmpty();
			case AlicaPackage.SYNCHRONISATION__TALK_TIMEOUT:
				return talkTimeout != TALK_TIMEOUT_EDEFAULT;
			case AlicaPackage.SYNCHRONISATION__SYNC_TIMEOUT:
				return syncTimeout != SYNC_TIMEOUT_EDEFAULT;
			case AlicaPackage.SYNCHRONISATION__FAIL_ON_SYNC_TIME_OUT:
				return failOnSyncTimeOut != FAIL_ON_SYNC_TIME_OUT_EDEFAULT;
		}
		return super.eIsSet(featureID);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public String toString() {
		if (eIsProxy()) return super.toString();

		StringBuffer result = new StringBuffer(super.toString());
		result.append(" (talkTimeout: ");
		result.append(talkTimeout);
		result.append(", syncTimeout: ");
		result.append(syncTimeout);
		result.append(", failOnSyncTimeOut: ");
		result.append(failOnSyncTimeOut);
		result.append(')');
		return result.toString();
	}

} //SynchronisationImpl
