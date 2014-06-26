/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica.impl;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.notify.NotificationChain;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.InternalEObject;
import org.eclipse.emf.ecore.impl.ENotificationImpl;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.PreCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>Transition</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.TransitionImpl#getMsg <em>Msg</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.TransitionImpl#getPreCondition <em>Pre Condition</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.TransitionImpl#getInState <em>In State</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.TransitionImpl#getOutState <em>Out State</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.TransitionImpl#getSynchronisation <em>Synchronisation</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class TransitionImpl extends PlanElementImpl implements Transition {
	/**
	 * The default value of the '{@link #getMsg() <em>Msg</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getMsg()
	 * @generated
	 * @ordered
	 */
	protected static final String MSG_EDEFAULT = "";

	/**
	 * The cached value of the '{@link #getMsg() <em>Msg</em>}' attribute.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getMsg()
	 * @generated
	 * @ordered
	 */
	protected String msg = MSG_EDEFAULT;

	/**
	 * The cached value of the '{@link #getPreCondition() <em>Pre Condition</em>}' containment reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getPreCondition()
	 * @generated
	 * @ordered
	 */
	protected PreCondition preCondition;

	/**
	 * The cached value of the '{@link #getInState() <em>In State</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getInState()
	 * @generated
	 * @ordered
	 */
	protected State inState;

	/**
	 * The cached value of the '{@link #getOutState() <em>Out State</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getOutState()
	 * @generated
	 * @ordered
	 */
	protected State outState;

	/**
	 * The cached value of the '{@link #getSynchronisation() <em>Synchronisation</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getSynchronisation()
	 * @generated
	 * @ordered
	 */
	protected Synchronisation synchronisation;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected TransitionImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return AlicaPackage.Literals.TRANSITION;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public String getMsg() {
		return msg;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setMsg(String newMsg) {
		String oldMsg = msg;
		msg = newMsg;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.TRANSITION__MSG, oldMsg, msg));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PreCondition getPreCondition() {
		return preCondition;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public NotificationChain basicSetPreCondition(PreCondition newPreCondition, NotificationChain msgs) {
		PreCondition oldPreCondition = preCondition;
		preCondition = newPreCondition;
		if (eNotificationRequired()) {
			ENotificationImpl notification = new ENotificationImpl(this, Notification.SET, AlicaPackage.TRANSITION__PRE_CONDITION, oldPreCondition, newPreCondition);
			if (msgs == null) msgs = notification; else msgs.add(notification);
		}
		return msgs;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setPreCondition(PreCondition newPreCondition) {
		if (newPreCondition != preCondition) {
			NotificationChain msgs = null;
			if (preCondition != null)
				msgs = ((InternalEObject)preCondition).eInverseRemove(this, EOPPOSITE_FEATURE_BASE - AlicaPackage.TRANSITION__PRE_CONDITION, null, msgs);
			if (newPreCondition != null)
				msgs = ((InternalEObject)newPreCondition).eInverseAdd(this, EOPPOSITE_FEATURE_BASE - AlicaPackage.TRANSITION__PRE_CONDITION, null, msgs);
			msgs = basicSetPreCondition(newPreCondition, msgs);
			if (msgs != null) msgs.dispatch();
		}
		else if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.TRANSITION__PRE_CONDITION, newPreCondition, newPreCondition));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public State getInState() {
		if (inState != null && inState.eIsProxy()) {
			InternalEObject oldInState = (InternalEObject)inState;
			inState = (State)eResolveProxy(oldInState);
			if (inState != oldInState) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.TRANSITION__IN_STATE, oldInState, inState));
			}
		}
		return inState;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public State basicGetInState() {
		return inState;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public NotificationChain basicSetInState(State newInState, NotificationChain msgs) {
		State oldInState = inState;
		inState = newInState;
		if (eNotificationRequired()) {
			ENotificationImpl notification = new ENotificationImpl(this, Notification.SET, AlicaPackage.TRANSITION__IN_STATE, oldInState, newInState);
			if (msgs == null) msgs = notification; else msgs.add(notification);
		}
		return msgs;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setInState(State newInState) {
		if (newInState != inState) {
			NotificationChain msgs = null;
			if (inState != null)
				msgs = ((InternalEObject)inState).eInverseRemove(this, AlicaPackage.STATE__OUT_TRANSITIONS, State.class, msgs);
			if (newInState != null)
				msgs = ((InternalEObject)newInState).eInverseAdd(this, AlicaPackage.STATE__OUT_TRANSITIONS, State.class, msgs);
			msgs = basicSetInState(newInState, msgs);
			if (msgs != null) msgs.dispatch();
		}
		else if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.TRANSITION__IN_STATE, newInState, newInState));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public State getOutState() {
		if (outState != null && outState.eIsProxy()) {
			InternalEObject oldOutState = (InternalEObject)outState;
			outState = (State)eResolveProxy(oldOutState);
			if (outState != oldOutState) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.TRANSITION__OUT_STATE, oldOutState, outState));
			}
		}
		return outState;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public State basicGetOutState() {
		return outState;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public NotificationChain basicSetOutState(State newOutState, NotificationChain msgs) {
		State oldOutState = outState;
		outState = newOutState;
		if (eNotificationRequired()) {
			ENotificationImpl notification = new ENotificationImpl(this, Notification.SET, AlicaPackage.TRANSITION__OUT_STATE, oldOutState, newOutState);
			if (msgs == null) msgs = notification; else msgs.add(notification);
		}
		return msgs;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setOutState(State newOutState) {
		if (newOutState != outState) {
			NotificationChain msgs = null;
			if (outState != null)
				msgs = ((InternalEObject)outState).eInverseRemove(this, AlicaPackage.STATE__IN_TRANSITIONS, State.class, msgs);
			if (newOutState != null)
				msgs = ((InternalEObject)newOutState).eInverseAdd(this, AlicaPackage.STATE__IN_TRANSITIONS, State.class, msgs);
			msgs = basicSetOutState(newOutState, msgs);
			if (msgs != null) msgs.dispatch();
		}
		else if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.TRANSITION__OUT_STATE, newOutState, newOutState));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Synchronisation getSynchronisation() {
		if (synchronisation != null && synchronisation.eIsProxy()) {
			InternalEObject oldSynchronisation = (InternalEObject)synchronisation;
			synchronisation = (Synchronisation)eResolveProxy(oldSynchronisation);
			if (synchronisation != oldSynchronisation) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.TRANSITION__SYNCHRONISATION, oldSynchronisation, synchronisation));
			}
		}
		return synchronisation;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Synchronisation basicGetSynchronisation() {
		return synchronisation;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public NotificationChain basicSetSynchronisation(Synchronisation newSynchronisation, NotificationChain msgs) {
		Synchronisation oldSynchronisation = synchronisation;
		synchronisation = newSynchronisation;
		if (eNotificationRequired()) {
			ENotificationImpl notification = new ENotificationImpl(this, Notification.SET, AlicaPackage.TRANSITION__SYNCHRONISATION, oldSynchronisation, newSynchronisation);
			if (msgs == null) msgs = notification; else msgs.add(notification);
		}
		return msgs;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setSynchronisation(Synchronisation newSynchronisation) {
		if (newSynchronisation != synchronisation) {
			NotificationChain msgs = null;
			if (synchronisation != null)
				msgs = ((InternalEObject)synchronisation).eInverseRemove(this, AlicaPackage.SYNCHRONISATION__SYNCHED_TRANSITIONS, Synchronisation.class, msgs);
			if (newSynchronisation != null)
				msgs = ((InternalEObject)newSynchronisation).eInverseAdd(this, AlicaPackage.SYNCHRONISATION__SYNCHED_TRANSITIONS, Synchronisation.class, msgs);
			msgs = basicSetSynchronisation(newSynchronisation, msgs);
			if (msgs != null) msgs.dispatch();
		}
		else if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.TRANSITION__SYNCHRONISATION, newSynchronisation, newSynchronisation));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public NotificationChain eInverseAdd(InternalEObject otherEnd, int featureID, NotificationChain msgs) {
		switch (featureID) {
			case AlicaPackage.TRANSITION__IN_STATE:
				if (inState != null)
					msgs = ((InternalEObject)inState).eInverseRemove(this, AlicaPackage.STATE__OUT_TRANSITIONS, State.class, msgs);
				return basicSetInState((State)otherEnd, msgs);
			case AlicaPackage.TRANSITION__OUT_STATE:
				if (outState != null)
					msgs = ((InternalEObject)outState).eInverseRemove(this, AlicaPackage.STATE__IN_TRANSITIONS, State.class, msgs);
				return basicSetOutState((State)otherEnd, msgs);
			case AlicaPackage.TRANSITION__SYNCHRONISATION:
				if (synchronisation != null)
					msgs = ((InternalEObject)synchronisation).eInverseRemove(this, AlicaPackage.SYNCHRONISATION__SYNCHED_TRANSITIONS, Synchronisation.class, msgs);
				return basicSetSynchronisation((Synchronisation)otherEnd, msgs);
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
			case AlicaPackage.TRANSITION__PRE_CONDITION:
				return basicSetPreCondition(null, msgs);
			case AlicaPackage.TRANSITION__IN_STATE:
				return basicSetInState(null, msgs);
			case AlicaPackage.TRANSITION__OUT_STATE:
				return basicSetOutState(null, msgs);
			case AlicaPackage.TRANSITION__SYNCHRONISATION:
				return basicSetSynchronisation(null, msgs);
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
			case AlicaPackage.TRANSITION__MSG:
				return getMsg();
			case AlicaPackage.TRANSITION__PRE_CONDITION:
				return getPreCondition();
			case AlicaPackage.TRANSITION__IN_STATE:
				if (resolve) return getInState();
				return basicGetInState();
			case AlicaPackage.TRANSITION__OUT_STATE:
				if (resolve) return getOutState();
				return basicGetOutState();
			case AlicaPackage.TRANSITION__SYNCHRONISATION:
				if (resolve) return getSynchronisation();
				return basicGetSynchronisation();
		}
		return super.eGet(featureID, resolve, coreType);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public void eSet(int featureID, Object newValue) {
		switch (featureID) {
			case AlicaPackage.TRANSITION__MSG:
				setMsg((String)newValue);
				return;
			case AlicaPackage.TRANSITION__PRE_CONDITION:
				setPreCondition((PreCondition)newValue);
				return;
			case AlicaPackage.TRANSITION__IN_STATE:
				setInState((State)newValue);
				return;
			case AlicaPackage.TRANSITION__OUT_STATE:
				setOutState((State)newValue);
				return;
			case AlicaPackage.TRANSITION__SYNCHRONISATION:
				setSynchronisation((Synchronisation)newValue);
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
			case AlicaPackage.TRANSITION__MSG:
				setMsg(MSG_EDEFAULT);
				return;
			case AlicaPackage.TRANSITION__PRE_CONDITION:
				setPreCondition((PreCondition)null);
				return;
			case AlicaPackage.TRANSITION__IN_STATE:
				setInState((State)null);
				return;
			case AlicaPackage.TRANSITION__OUT_STATE:
				setOutState((State)null);
				return;
			case AlicaPackage.TRANSITION__SYNCHRONISATION:
				setSynchronisation((Synchronisation)null);
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
			case AlicaPackage.TRANSITION__MSG:
				return MSG_EDEFAULT == null ? msg != null : !MSG_EDEFAULT.equals(msg);
			case AlicaPackage.TRANSITION__PRE_CONDITION:
				return preCondition != null;
			case AlicaPackage.TRANSITION__IN_STATE:
				return inState != null;
			case AlicaPackage.TRANSITION__OUT_STATE:
				return outState != null;
			case AlicaPackage.TRANSITION__SYNCHRONISATION:
				return synchronisation != null;
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
		result.append(" (msg: ");
		result.append(msg);
		result.append(')');
		return result.toString();
	}

} //TransitionImpl
