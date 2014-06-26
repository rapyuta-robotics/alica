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
import org.eclipse.emf.ecore.util.InternalEList;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskRepository;

/**
 * <!-- begin-user-doc -->
 * An implementation of the model object '<em><b>Task Repository</b></em>'.
 * <!-- end-user-doc -->
 * <p>
 * The following features are implemented:
 * <ul>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.TaskRepositoryImpl#getTasks <em>Tasks</em>}</li>
 *   <li>{@link de.uni_kassel.vs.cn.planDesigner.alica.impl.TaskRepositoryImpl#getDefaultTask <em>Default Task</em>}</li>
 * </ul>
 * </p>
 *
 * @generated
 */
public class TaskRepositoryImpl extends PlanElementImpl implements TaskRepository {
	/**
	 * The cached value of the '{@link #getTasks() <em>Tasks</em>}' containment reference list.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getTasks()
	 * @generated
	 * @ordered
	 */
	protected EList<Task> tasks;

	/**
	 * The cached value of the '{@link #getDefaultTask() <em>Default Task</em>}' reference.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @see #getDefaultTask()
	 * @generated
	 * @ordered
	 */
	protected Task defaultTask;

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected TaskRepositoryImpl() {
		super();
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected EClass eStaticClass() {
		return AlicaPackage.Literals.TASK_REPOSITORY;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public EList<Task> getTasks() {
		if (tasks == null) {
			tasks = new EObjectContainmentEList<Task>(Task.class, this, AlicaPackage.TASK_REPOSITORY__TASKS);
		}
		return tasks;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Task getDefaultTask() {
		if (defaultTask != null && defaultTask.eIsProxy()) {
			InternalEObject oldDefaultTask = (InternalEObject)defaultTask;
			defaultTask = (Task)eResolveProxy(oldDefaultTask);
			if (defaultTask != oldDefaultTask) {
				if (eNotificationRequired())
					eNotify(new ENotificationImpl(this, Notification.RESOLVE, AlicaPackage.TASK_REPOSITORY__DEFAULT_TASK, oldDefaultTask, defaultTask));
			}
		}
		return defaultTask;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public Task basicGetDefaultTask() {
		return defaultTask;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public void setDefaultTask(Task newDefaultTask) {
		Task oldDefaultTask = defaultTask;
		defaultTask = newDefaultTask;
		if (eNotificationRequired())
			eNotify(new ENotificationImpl(this, Notification.SET, AlicaPackage.TASK_REPOSITORY__DEFAULT_TASK, oldDefaultTask, defaultTask));
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated NOT
	 */
	public Task createDefaultTask() {
		Task defaultTask = AlicaFactory.eINSTANCE.createTask();
		defaultTask.setName("DefaultTask");
		defaultTask.setDescription("Default Task.");
		return defaultTask;
	}

	/**
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public NotificationChain eInverseRemove(InternalEObject otherEnd, int featureID, NotificationChain msgs) {
		switch (featureID) {
			case AlicaPackage.TASK_REPOSITORY__TASKS:
				return ((InternalEList<?>)getTasks()).basicRemove(otherEnd, msgs);
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
			case AlicaPackage.TASK_REPOSITORY__TASKS:
				return getTasks();
			case AlicaPackage.TASK_REPOSITORY__DEFAULT_TASK:
				if (resolve) return getDefaultTask();
				return basicGetDefaultTask();
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
			case AlicaPackage.TASK_REPOSITORY__TASKS:
				getTasks().clear();
				getTasks().addAll((Collection<? extends Task>)newValue);
				return;
			case AlicaPackage.TASK_REPOSITORY__DEFAULT_TASK:
				setDefaultTask((Task)newValue);
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
			case AlicaPackage.TASK_REPOSITORY__TASKS:
				getTasks().clear();
				return;
			case AlicaPackage.TASK_REPOSITORY__DEFAULT_TASK:
				setDefaultTask((Task)null);
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
			case AlicaPackage.TASK_REPOSITORY__TASKS:
				return tasks != null && !tasks.isEmpty();
			case AlicaPackage.TASK_REPOSITORY__DEFAULT_TASK:
				return defaultTask != null;
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
			case AlicaPackage.TASK_REPOSITORY___CREATE_DEFAULT_TASK:
				return createDefaultTask();
		}
		return super.eInvoke(operationID, arguments);
	}

} //TaskRepositoryImpl
