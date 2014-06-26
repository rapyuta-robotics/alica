/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica.impl;

import java.lang.reflect.InvocationTargetException;
import java.util.Collection;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.notify.NotificationChain;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.InternalEObject;
import org.eclipse.emf.ecore.impl.ENotificationImpl;
import org.eclipse.emf.ecore.util.EObjectContainmentEList;
import org.eclipse.emf.ecore.util.EObjectResolvingEList;
import org.eclipse.emf.ecore.util.EObjectWithInverseResolvingEList;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.ecore.util.InternalEList;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>State</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.StateImpl#getPlans <em>Plans</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.StateImpl#getInPlan <em>In Plan</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.StateImpl#getParametrisation <em>Parametrisation</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.StateImpl#getInTransitions <em>In Transitions</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.StateImpl#getOutTransitions <em>Out Transitions</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.StateImpl#getEntryPoint <em>Entry Point</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class StateImpl extends PlanElementImpl implements State {
	/**
	 * The cached value of the '{@link #getPlans() <em>Plans</em>}' reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getPlans()
	 * @generated
	 * @ordered
	 */
	protected EList<AbstractPlan> plans;

	/**
	 * The cached value of the '{@link #getParametrisation() <em>Parametrisation</em>}' containment reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getParametrisation()
	 * @generated
	 * @ordered
	 */
	protected EList<Parametrisation> parametrisation;

	/**
	 * The cached value of the '{@link #getInTransitions() <em>In Transitions</em>}' reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getInTransitions()
	 * @generated
	 * @ordered
	 */
	protected EList<Transition> inTransitions;

	/**
	 * The cached value of the '{@link #getOutTransitions() <em>Out Transitions</em>}' reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getOutTransitions()
	 * @generated
	 * @ordered
	 */
	protected EList<Transition> outTransitions;

	/**
	 * The cached value of the '{@link #getEntryPoint() <em>Entry Point</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getEntryPoint()
	 * @generated
	 * @ordered
	 */
	protected EntryPoint entryPoint;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected StateImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return AlicaPackage.Literals.STATE;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<AbstractPlan> getPlans() {
		if (plans == null) {
			plans = new EObjectResolvingEList<AbstractPlan>(AbstractPlan.class, this, AlicaPackage.STATE__PLANS);
		}
		return plans;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Plan getInPlan() {
		if (eContainerFeatureID() != AlicaPackage.STATE__IN_PLAN) return null;
		return (Plan)eInternalContainer();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public NotificationChain basicSetInPlan(Plan newInPlan, NotificationChain msgs) {
		msgs = eBasicSetContainer((InternalEObject)newInPlan, AlicaPackage.STATE__IN_PLAN, msgs);
		return msgs;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setInPlan(Plan newInPlan) {
		if (newInPlan != eInternalContainer() || (eContainerFeatureID() != AlicaPackage.STATE__IN_PLAN && newInPlan != null)) {
			if (EcoreUtil.isAncestor(this, newInPlan))
				throw new IllegalArgumentException("Recursive containment not allowed for " + toString());
			NotificationChain msgs = null;
			if (eInternalContainer() != null)
				msgs = eBasicRemoveFromContainer(msgs);
			if (newInPlan != null)
				msgs = ((InternalEObject)newInPlan).eInverseAdd(this, AlicaPackage.PLAN__STATES, Plan.class, msgs);
			msgs = basicSetInPlan(newInPlan, msgs);
			if (msgs != null) msgs.dispatch();
		}
		else if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.STATE__IN_PLAN, newInPlan, newInPlan));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<Parametrisation> getParametrisation() {
		if (parametrisation == null) {
			parametrisation = new EObjectContainmentEList<Parametrisation>(Parametrisation.class, this, AlicaPackage.STATE__PARAMETRISATION);
		}
		return parametrisation;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<Transition> getInTransitions() {
		if (inTransitions == null) {
			inTransitions = new EObjectWithInverseResolvingEList<Transition>(Transition.class, this, AlicaPackage.STATE__IN_TRANSITIONS, AlicaPackage.TRANSITION__OUT_STATE);
		}
		return inTransitions;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<Transition> getOutTransitions() {
		if (outTransitions == null) {
			outTransitions = new EObjectWithInverseResolvingEList<Transition>(Transition.class, this, AlicaPackage.STATE__OUT_TRANSITIONS, AlicaPackage.TRANSITION__IN_STATE);
		}
		return outTransitions;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EntryPoint getEntryPoint() {
		if (entryPoint != null && entryPoint.eIsProxy()) {
			InternalEObject oldEntryPoint = (InternalEObject)entryPoint;
			entryPoint = (EntryPoint)eResolveProxy(oldEntryPoint);
			if (entryPoint != oldEntryPoint) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.STATE__ENTRY_POINT, oldEntryPoint, entryPoint));
			}
		}
		return entryPoint;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EntryPoint basicGetEntryPoint() {
		return entryPoint;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public NotificationChain basicSetEntryPoint(EntryPoint newEntryPoint, NotificationChain msgs) {
		EntryPoint oldEntryPoint = entryPoint;
		entryPoint = newEntryPoint;
		if (eNotificationRequired()) {
			ENotificationImpl notification = new ENotificationImpl(this, Notification.SET, AlicaPackage.STATE__ENTRY_POINT, oldEntryPoint, newEntryPoint);
			if (msgs == null) msgs = notification; else msgs.add(notification);
		}
		return msgs;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setEntryPoint(EntryPoint newEntryPoint) {
		if (newEntryPoint != entryPoint) {
			NotificationChain msgs = null;
			if (entryPoint != null)
				msgs = ((InternalEObject)entryPoint).eInverseRemove(this, AlicaPackage.ENTRY_POINT__STATE, EntryPoint.class, msgs);
			if (newEntryPoint != null)
				msgs = ((InternalEObject)newEntryPoint).eInverseAdd(this, AlicaPackage.ENTRY_POINT__STATE, EntryPoint.class, msgs);
			msgs = basicSetEntryPoint(newEntryPoint, msgs);
			if (msgs != null) msgs.dispatch();
		}
		else if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.STATE__ENTRY_POINT, newEntryPoint, newEntryPoint));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated NOT
	 */
	public void ensureParametrisationConsistency() {
		boolean dirty = false;
		if (this.parametrisation==null) return;
		for(int i=0; i<this.parametrisation.size(); i++) {
			boolean remove = false;
			if(!this.getInPlan().getVars().contains(this.parametrisation.get(i).getVar())) {
				remove = true;
			} else if (!this.parametrisation.get(i).getSubplan().getVars().contains(this.parametrisation.get(i).getSubvar())) {
				remove = true;
			}
			else if(!this.getPlans().contains(this.parametrisation.get(i).getSubplan())) {
				remove = true;
			}
			if (remove) {
				this.parametrisation.remove(i);
				i--;
				dirty = true;
			}
		}
		if (dirty) {
			//eNotify(new ENotificationImpl(this, Notification.REMOVE_MANY, AlicaPackage.STATE__PARAMETRISATION,this.parametrisation,this.parametrisation,true));
		}
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
			case AlicaPackage.STATE__IN_PLAN:
				if (eInternalContainer() != null)
					msgs = eBasicRemoveFromContainer(msgs);
				return basicSetInPlan((Plan)otherEnd, msgs);
			case AlicaPackage.STATE__IN_TRANSITIONS:
				return ((InternalEList<InternalEObject>)(InternalEList<?>)getInTransitions()).basicAdd(otherEnd, msgs);
			case AlicaPackage.STATE__OUT_TRANSITIONS:
				return ((InternalEList<InternalEObject>)(InternalEList<?>)getOutTransitions()).basicAdd(otherEnd, msgs);
			case AlicaPackage.STATE__ENTRY_POINT:
				if (entryPoint != null)
					msgs = ((InternalEObject)entryPoint).eInverseRemove(this, AlicaPackage.ENTRY_POINT__STATE, EntryPoint.class, msgs);
				return basicSetEntryPoint((EntryPoint)otherEnd, msgs);
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
			case AlicaPackage.STATE__IN_PLAN:
				return basicSetInPlan(null, msgs);
			case AlicaPackage.STATE__PARAMETRISATION:
				return ((InternalEList<?>)getParametrisation()).basicRemove(otherEnd, msgs);
			case AlicaPackage.STATE__IN_TRANSITIONS:
				return ((InternalEList<?>)getInTransitions()).basicRemove(otherEnd, msgs);
			case AlicaPackage.STATE__OUT_TRANSITIONS:
				return ((InternalEList<?>)getOutTransitions()).basicRemove(otherEnd, msgs);
			case AlicaPackage.STATE__ENTRY_POINT:
				return basicSetEntryPoint(null, msgs);
		}
		return super.eInverseRemove(otherEnd, featureID, msgs);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public NotificationChain eBasicRemoveFromContainerFeature(NotificationChain msgs) {
		switch (eContainerFeatureID()) {
			case AlicaPackage.STATE__IN_PLAN:
				return eInternalContainer().eInverseRemove(this, AlicaPackage.PLAN__STATES, Plan.class, msgs);
		}
		return super.eBasicRemoveFromContainerFeature(msgs);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public Object eGet(int featureID, boolean resolve, boolean coreType) {
		switch (featureID) {
			case AlicaPackage.STATE__PLANS:
				return getPlans();
			case AlicaPackage.STATE__IN_PLAN:
				return getInPlan();
			case AlicaPackage.STATE__PARAMETRISATION:
				return getParametrisation();
			case AlicaPackage.STATE__IN_TRANSITIONS:
				return getInTransitions();
			case AlicaPackage.STATE__OUT_TRANSITIONS:
				return getOutTransitions();
			case AlicaPackage.STATE__ENTRY_POINT:
				if (resolve) return getEntryPoint();
				return basicGetEntryPoint();
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
			case AlicaPackage.STATE__PLANS:
				getPlans().clear();
				getPlans().addAll((Collection<? extends AbstractPlan>)newValue);
				return;
			case AlicaPackage.STATE__IN_PLAN:
				setInPlan((Plan)newValue);
				return;
			case AlicaPackage.STATE__PARAMETRISATION:
				getParametrisation().clear();
				getParametrisation().addAll((Collection<? extends Parametrisation>)newValue);
				return;
			case AlicaPackage.STATE__IN_TRANSITIONS:
				getInTransitions().clear();
				getInTransitions().addAll((Collection<? extends Transition>)newValue);
				return;
			case AlicaPackage.STATE__OUT_TRANSITIONS:
				getOutTransitions().clear();
				getOutTransitions().addAll((Collection<? extends Transition>)newValue);
				return;
			case AlicaPackage.STATE__ENTRY_POINT:
				setEntryPoint((EntryPoint)newValue);
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
			case AlicaPackage.STATE__PLANS:
				getPlans().clear();
				return;
			case AlicaPackage.STATE__IN_PLAN:
				setInPlan((Plan)null);
				return;
			case AlicaPackage.STATE__PARAMETRISATION:
				getParametrisation().clear();
				return;
			case AlicaPackage.STATE__IN_TRANSITIONS:
				getInTransitions().clear();
				return;
			case AlicaPackage.STATE__OUT_TRANSITIONS:
				getOutTransitions().clear();
				return;
			case AlicaPackage.STATE__ENTRY_POINT:
				setEntryPoint((EntryPoint)null);
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
			case AlicaPackage.STATE__PLANS:
				return plans != null && !plans.isEmpty();
			case AlicaPackage.STATE__IN_PLAN:
				return getInPlan() != null;
			case AlicaPackage.STATE__PARAMETRISATION:
				return parametrisation != null && !parametrisation.isEmpty();
			case AlicaPackage.STATE__IN_TRANSITIONS:
				return inTransitions != null && !inTransitions.isEmpty();
			case AlicaPackage.STATE__OUT_TRANSITIONS:
				return outTransitions != null && !outTransitions.isEmpty();
			case AlicaPackage.STATE__ENTRY_POINT:
				return entryPoint != null;
		}
		return super.eIsSet(featureID);
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public Object eInvoke(int operationID, EList<?> arguments) throws InvocationTargetException {
		switch (operationID) {
			case AlicaPackage.STATE___ENSURE_PARAMETRISATION_CONSISTENCY:
				ensureParametrisationConsistency();
				return null;
		}
		return super.eInvoke(operationID, arguments);
	}

} //StateImpl
