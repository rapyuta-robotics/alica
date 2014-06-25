/**
 */
package de.uni_kassel.vs.cn.planDesigner.alica.provider;


import java.util.Collection;
import java.util.List;

import org.eclipse.emf.common.notify.AdapterFactory;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.edit.provider.ComposeableAdapterFactory;
import org.eclipse.emf.edit.provider.IEditingDomainItemProvider;
import org.eclipse.emf.edit.provider.IItemLabelProvider;
import org.eclipse.emf.edit.provider.IItemPropertyDescriptor;
import org.eclipse.emf.edit.provider.IItemPropertySource;
import org.eclipse.emf.edit.provider.IStructuredItemContentProvider;
import org.eclipse.emf.edit.provider.ITreeItemContentProvider;

import org.eclipse.emf.edit.provider.ItemPropertyDescriptor;
import org.eclipse.emf.edit.provider.ViewerNotification;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;

/**
 * This is the item provider adapter for a {@link de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem} object.
 * <!-- begin-user-doc -->
 * <!-- end-user-doc -->
 * @generated
 */
public class PlanningProblemItemProvider
	extends AbstractPlanItemProvider
	implements
		IEditingDomainItemProvider,
		IStructuredItemContentProvider,
		ITreeItemContentProvider,
		IItemLabelProvider,
		IItemPropertySource {
	/**
	 * This constructs an instance from a factory and a notifier.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	public PlanningProblemItemProvider(AdapterFactory adapterFactory) {
		super(adapterFactory);
	}

	/**
	 * This returns the property descriptors for the adapted class.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public List<IItemPropertyDescriptor> getPropertyDescriptors(Object object) {
		if (itemPropertyDescriptors == null) {
			super.getPropertyDescriptors(object);

			addPlansPropertyDescriptor(object);
			addPlannerPropertyDescriptor(object);
			addAlternativePlanPropertyDescriptor(object);
			addWaitPlanPropertyDescriptor(object);
			addUpdateRatePropertyDescriptor(object);
			addDistributeProblemPropertyDescriptor(object);
			addPlanningTypePropertyDescriptor(object);
			addRequirementsPropertyDescriptor(object);
			addPlannerParamsPropertyDescriptor(object);
		}
		return itemPropertyDescriptors;
	}

	/**
	 * This adds a property descriptor for the Plans feature.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected void addPlansPropertyDescriptor(Object object) {
		itemPropertyDescriptors.add
			(createItemPropertyDescriptor
				(((ComposeableAdapterFactory)adapterFactory).getRootAdapterFactory(),
				 getResourceLocator(),
				 getString("_UI_PlanningProblem_plans_feature"),
				 getString("_UI_PropertyDescriptor_description", "_UI_PlanningProblem_plans_feature", "_UI_PlanningProblem_type"),
				 AlicaPackage.Literals.PLANNING_PROBLEM__PLANS,
				 true,
				 false,
				 true,
				 null,
				 null,
				 null));
	}

	/**
	 * This adds a property descriptor for the Planner feature.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected void addPlannerPropertyDescriptor(Object object) {
		itemPropertyDescriptors.add
			(createItemPropertyDescriptor
				(((ComposeableAdapterFactory)adapterFactory).getRootAdapterFactory(),
				 getResourceLocator(),
				 getString("_UI_PlanningProblem_planner_feature"),
				 getString("_UI_PropertyDescriptor_description", "_UI_PlanningProblem_planner_feature", "_UI_PlanningProblem_type"),
				 AlicaPackage.Literals.PLANNING_PROBLEM__PLANNER,
				 true,
				 false,
				 true,
				 null,
				 null,
				 null));
	}

	/**
	 * This adds a property descriptor for the Alternative Plan feature.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected void addAlternativePlanPropertyDescriptor(Object object) {
		itemPropertyDescriptors.add
			(createItemPropertyDescriptor
				(((ComposeableAdapterFactory)adapterFactory).getRootAdapterFactory(),
				 getResourceLocator(),
				 getString("_UI_PlanningProblem_alternativePlan_feature"),
				 getString("_UI_PropertyDescriptor_description", "_UI_PlanningProblem_alternativePlan_feature", "_UI_PlanningProblem_type"),
				 AlicaPackage.Literals.PLANNING_PROBLEM__ALTERNATIVE_PLAN,
				 true,
				 false,
				 true,
				 null,
				 null,
				 null));
	}

	/**
	 * This adds a property descriptor for the Wait Plan feature.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected void addWaitPlanPropertyDescriptor(Object object) {
		itemPropertyDescriptors.add
			(createItemPropertyDescriptor
				(((ComposeableAdapterFactory)adapterFactory).getRootAdapterFactory(),
				 getResourceLocator(),
				 getString("_UI_PlanningProblem_waitPlan_feature"),
				 getString("_UI_PropertyDescriptor_description", "_UI_PlanningProblem_waitPlan_feature", "_UI_PlanningProblem_type"),
				 AlicaPackage.Literals.PLANNING_PROBLEM__WAIT_PLAN,
				 true,
				 false,
				 true,
				 null,
				 null,
				 null));
	}

	/**
	 * This adds a property descriptor for the Update Rate feature.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected void addUpdateRatePropertyDescriptor(Object object) {
		itemPropertyDescriptors.add
			(createItemPropertyDescriptor
				(((ComposeableAdapterFactory)adapterFactory).getRootAdapterFactory(),
				 getResourceLocator(),
				 getString("_UI_PlanningProblem_updateRate_feature"),
				 getString("_UI_PropertyDescriptor_description", "_UI_PlanningProblem_updateRate_feature", "_UI_PlanningProblem_type"),
				 AlicaPackage.Literals.PLANNING_PROBLEM__UPDATE_RATE,
				 true,
				 false,
				 false,
				 ItemPropertyDescriptor.INTEGRAL_VALUE_IMAGE,
				 null,
				 null));
	}

	/**
	 * This adds a property descriptor for the Distribute Problem feature.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected void addDistributeProblemPropertyDescriptor(Object object) {
		itemPropertyDescriptors.add
			(createItemPropertyDescriptor
				(((ComposeableAdapterFactory)adapterFactory).getRootAdapterFactory(),
				 getResourceLocator(),
				 getString("_UI_PlanningProblem_distributeProblem_feature"),
				 getString("_UI_PropertyDescriptor_description", "_UI_PlanningProblem_distributeProblem_feature", "_UI_PlanningProblem_type"),
				 AlicaPackage.Literals.PLANNING_PROBLEM__DISTRIBUTE_PROBLEM,
				 true,
				 false,
				 false,
				 ItemPropertyDescriptor.BOOLEAN_VALUE_IMAGE,
				 null,
				 null));
	}

	/**
	 * This adds a property descriptor for the Planning Type feature.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected void addPlanningTypePropertyDescriptor(Object object) {
		itemPropertyDescriptors.add
			(createItemPropertyDescriptor
				(((ComposeableAdapterFactory)adapterFactory).getRootAdapterFactory(),
				 getResourceLocator(),
				 getString("_UI_PlanningProblem_planningType_feature"),
				 getString("_UI_PropertyDescriptor_description", "_UI_PlanningProblem_planningType_feature", "_UI_PlanningProblem_type"),
				 AlicaPackage.Literals.PLANNING_PROBLEM__PLANNING_TYPE,
				 true,
				 false,
				 false,
				 ItemPropertyDescriptor.GENERIC_VALUE_IMAGE,
				 null,
				 null));
	}

	/**
	 * This adds a property descriptor for the Requirements feature.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected void addRequirementsPropertyDescriptor(Object object) {
		itemPropertyDescriptors.add
			(createItemPropertyDescriptor
				(((ComposeableAdapterFactory)adapterFactory).getRootAdapterFactory(),
				 getResourceLocator(),
				 getString("_UI_PlanningProblem_requirements_feature"),
				 getString("_UI_PropertyDescriptor_description", "_UI_PlanningProblem_requirements_feature", "_UI_PlanningProblem_type"),
				 AlicaPackage.Literals.PLANNING_PROBLEM__REQUIREMENTS,
				 true,
				 false,
				 false,
				 ItemPropertyDescriptor.GENERIC_VALUE_IMAGE,
				 null,
				 null));
	}

	/**
	 * This adds a property descriptor for the Planner Params feature.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected void addPlannerParamsPropertyDescriptor(Object object) {
		itemPropertyDescriptors.add
			(createItemPropertyDescriptor
				(((ComposeableAdapterFactory)adapterFactory).getRootAdapterFactory(),
				 getResourceLocator(),
				 getString("_UI_PlanningProblem_plannerParams_feature"),
				 getString("_UI_PropertyDescriptor_description", "_UI_PlanningProblem_plannerParams_feature", "_UI_PlanningProblem_type"),
				 AlicaPackage.Literals.PLANNING_PROBLEM__PLANNER_PARAMS,
				 true,
				 false,
				 false,
				 ItemPropertyDescriptor.GENERIC_VALUE_IMAGE,
				 null,
				 null));
	}

	/**
	 * This returns PlanningProblem.gif.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public Object getImage(Object object) {
		return overlayImage(object, getResourceLocator().getImage("full/obj16/PlanningProblem"));
	}

	/**
	 * This returns the label text for the adapted class.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public String getText(Object object) {
		String label = ((PlanningProblem)object).getName();
		return label == null || label.length() == 0 ?
			getString("_UI_PlanningProblem_type") :
			getString("_UI_PlanningProblem_type") + " " + label;
	}

	/**
	 * This handles model notifications by calling {@link #updateChildren} to update any cached
	 * children and by creating a viewer notification, which it passes to {@link #fireNotifyChanged}.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	public void notifyChanged(Notification notification) {
		updateChildren(notification);

		switch (notification.getFeatureID(PlanningProblem.class)) {
			case AlicaPackage.PLANNING_PROBLEM__UPDATE_RATE:
			case AlicaPackage.PLANNING_PROBLEM__DISTRIBUTE_PROBLEM:
			case AlicaPackage.PLANNING_PROBLEM__PLANNING_TYPE:
			case AlicaPackage.PLANNING_PROBLEM__REQUIREMENTS:
			case AlicaPackage.PLANNING_PROBLEM__PLANNER_PARAMS:
				fireNotifyChanged(new ViewerNotification(notification, notification.getNotifier(), false, true));
				return;
		}
		super.notifyChanged(notification);
	}

	/**
	 * This adds {@link org.eclipse.emf.edit.command.CommandParameter}s describing the children
	 * that can be created under this object.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	@Override
	protected void collectNewChildDescriptors(Collection<Object> newChildDescriptors, Object object) {
		super.collectNewChildDescriptors(newChildDescriptors, object);
	}

}
