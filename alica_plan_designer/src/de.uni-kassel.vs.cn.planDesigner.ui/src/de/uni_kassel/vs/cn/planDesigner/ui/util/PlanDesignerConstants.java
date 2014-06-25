// Copyright 2009 Distributed Systems Group, University of Kassel
// This program is distributed under the GNU Lesser General Public License (LGPL).
//
// This file is part of the Carpe Noctem Software Framework.
//
//    The Carpe Noctem Software Framework is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Lesser General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    The Carpe Noctem Software Framework is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Lesser General Public License for more details.
package de.uni_kassel.vs.cn.planDesigner.ui.util;

public interface PlanDesignerConstants {
	
	// ID's for views
	public static final String ID_PML_REPOSITORY = "de.uni_kassel.vs.cn.planDesigner.ui.views.RepositoryView";
	
	// ID's for wizards
	public static final String ID_PML_NEW_PLAN_WIZARD = "de.uni_kassel.vs.cn.planDesigner.ui.wizards.PlanmodellerNewPlanWizard";
	public static final String ID_PML_NEW_PLAN_TYPE_WIZARD = "de.uni_kassel.vs.cn.planDesigner.ui.wizards.PlanmodellerNewPlantypeWizard";
	public static final String ID_PML_NEW_BEHAVIOUR_WIZARD = "de.uni_kassel.vs.cn.planDesigner.ui.wizards.PlanmodellerNewBehaviourWizard";
	public static final String ID_PML_NEW_ROLESET_WIZARD = "de.uni_kassel.vs.cn.planDesigner.ui.wizards.PlanmodellerNewRolesetWizard";
	public static final String ID_PML_NEW_PLANNING_PROBLEM_WIZARD = "de.uni_kassel.vs.cn.planDesigner.ui.wizards.PlanmodellerNewPlanningProblemWizard";
	public static final String ID_PML_NEW_PLANNER_WIZARD = "de.uni_kassel.vs.cn.planDesigner.ui.wizards.PlanmodellerNewPlannerWizard";
	public static final String ID_PML_NEW_DOMAIN_DESCRIPTION_WIZARD = "de.uni_kassel.vs.cn.planDesigner.ui.wizards.PlanmodellerNewDomainDescriptionWizard";
	
	// The plan editor ID=
	public static final String PLAN_EDITOR_ID = "de.uni_kassel.vs.cn.planDesigner.ui.pmlEditor";
	
	// The roleset editor ID
	public static final String ROLESET_EDITOR_ID = "de.uni_kassel.vs.cn.planDesigner.ui.roleSetEditor";
	
	// The repository viewer ID
	public static final String PML_REPOSITORY_ID = "de.uni_kassel.vs.cn.planDesigner.ui.views.RepositoryView";
	
	// The Plan Designer explorer viewer ID
	public static final String PML_EXPLORER_ID = "de.uni_kassel.vs.cn.planDesigner.ui.views.pdexplorer";
	
	// The ID for the TransactionalEditinDomain
	public static final String PML_TRANSACTIONAL_EDITING_DOMAIN_ID = "de.uni_kassel.vs.cn.planDesigner.uitransaction.editingDomain";
	
	// The ID for the PlanDesigner Perspective
	public static final String PML_PERSPECTIVE_ID = "de.uni_kassel.vs.cn.planDesigner.ui.pmleditorPerspective";

	// Image path
	public static final String ICON_PATH = "icons/";
	
	// filename for global role definitionset file
	public static final String ROLE_DEFINITION_FILE = "rolesDefinition.rdefset";
	
	// filename for global CAPABILITY definitionset file
	public static final String CAPABILITY_DEFINITION_FILE = "capabilitiesDefinition.cdefset";
	
	// filename for the taskrepository
	public static final String TASK_REPOSITORY_FILE = "taskrepository.tsk";
	
	public static final String UTILITY_REPOSITORY_FILE = "utilityRepository.repo";
	
	// Some keys used within the preferencestore
	public static final String PREF_ROLE_DEFINITION_CONTAINER = "prefRoleDefinitionContainer";
	public static final String PREF_CAPABILITY_DEFINITION_CONTAINER = "prefCapabilityDefinitionContainer";
	public static final String PREF_CODEGEN_BASE_PATH = "prefCodegenPath";
	public static final String PREF_CODEGEN_GOALS_BASE_PATH = "prefCodegenGoalPath";
	public static final String PREF_MISC_PROJECT_WORKSPACE_PATH = "miscPath";
	public static final String PREF_PP_PATH = "planningProblemPath";
	public static final String PREF_GOAL_EXPR_PATH = "goalexpressionPath";

	// Some colors, used within the editor
	public static final String PLAN_STATE_BACKGROUND_COLOR = "planStateBackgroundColor";
	public 	static final String STATE_BACKGROUND_COLOR = "stateBackgroundColor";
	public static final String ENTRY_POINT_BACKGROUND_COLOR = "entryPointBackgroundColor";
	public static final String SUCCESS_POINT_BACKGROUND_COLOR = "successPointBackgroundColor";
	public static final String FAILURE_POINT_BACKGROUND_COLOR = "failurePointBackgroundColor";
	public static final String PLAN_TYPE_POINT_BACKGROUND_COLOR = "planTypeBackgroundColor";
	public static final String PLAN_LABEL_BACKGROUND_COLOR = "planLabelBackgroundColor";
	public static final String MASTER_PLAN_LABEL_BACKGROUND_COLOR = "masterPlanLabelBackgroundColor";
	public static final String SUCCESSTRANSITION_FOREGROUND_COLOR = "successTransitionForegroundColor";
	public static final String FAILURERANSITION_FOREGROUND_COLOR = "failureTransitionForegroundColor";
	
	/**  Color for range -1 <= x < 0 */
	public static final String PRIORITY_0_COLOR = "priority0Color";
	
	/** Color for 0 <= x < 0.5 */ 
	public static final String PRIORITY_1_COLOR = "priority1Color";
	
	/** Color for 0.5 < x <= 1 */
	public static final String PRIORITY_2_COLOR = "priority2Color";
	
	/** Color for default value 0.5 */
	public static final String PRIORITY_DEFAULT_COLOR = "priorityDefaultColor";
	
	// Some icons used within the editor
	public static final String ICON_CHARACTERISTIC_16 = "characteristic16x16.png";
	public static final String ICON_CHARACTERISTIC_32 = "characteristic32x32.png";
	public static final String ICON_CAPVALUE_16 = "characteristic16x16.png";
	public static final String ICON_CAPVALUE_32 = "characteristic32x32.png";
	
	public static final String ICON_CN_2008 = "cn200816x16.png";
	
	public static final String ICON_CSHARP_16 = "csharp16x16.png";
	public static final String ICON_CSHARP_24 = "csharp24x24.png";
	
	public static final String ICON_BEHAVIOUR_16 = "behaviour16x16.png";
	public static final String ICON_BEHAVIOUR_24 = "behaviour24x24.png";
	
	public static final String ICON_BEHAVIOUR_CONFIGURATION_16 = "behaviourConfiguration16x16.png";
	public static final String ICON_BEHAVIOUR_CONFIGURATION_24 = "behaviourConfiguration24x24.png";
	
	public static final String ICON_PLANTYPE_16 = "planTyp16x16.png";
	public static final String ICON_PLANTYPE_24 = "planTyp24x24.png";
	
	public static final String ICON_PLANNING_PROBLEM_16 = "planningProblem16x16.png";
	public static final String ICON_PLANNING_PROBLEM_24 = "planningProblem24x24.png";
	
	public static final String ICON_PLAN_16 = "plan16x16.png";
	public static final String ICON_PLAN_24 = "plan24x24.png";
	
	public static final String ICON_PLANNER_16 = "csharp16x16.png";

	
	public static final String ICON_POST_CONDITION_12 = "result12x12.png";
	public static final String ICON_POST_CONDITION_16 = "result16x16.png";
	public static final String ICON_POST_CONDITION_20 = "result20x20.png";
	public static final String ICON_POST_CONDITION_24 = "result24x24.png";
	
	public static final String ICON_PRE_CONDITION_12 = "precondition12x12.png";
	public static final String ICON_PRE_CONDITION_16 = "precondition16x16.png";
	public static final String ICON_PRE_CONDITION_20 = "precondition20x20.png";
	public static final String ICON_PRE_CONDITION_24 = "precondition24x24.png";
	
	public static final String ICON_ROLE_16 = "role16x16.png";
	public static final String ICON_ROLE_32 = "role32x32.png";
	
	public static final String ICON_ROLE_DEFINITION_16 = "roleDefinition16x16.png";
	public static final String ICON_ROLE_DEFINITION_32 = "roleDefinition32x32.png";
	public static final String ICON_ROLE_DEFINITION_64 = "roleDefinition64x64.png";
	public static final String ICON_ROLE_DEFINITION_128 = "roleDefinition128x128.png";
	
	public static final String ICON_CAPABILITY_16 = "role16x16.png";
	public static final String ICON_CAPABILITY_32 = "role32x32.png";
	
	public static final String ICON_CAPABILITY_DEFINITION_16 = "roleDefinition16x16.png";
	public static final String ICON_CAPABILITY_DEFINITION_32 = "roleDefinition32x32.png";
	public static final String ICON_CAPABILITY_DEFINITION_64 = "roleDefinition64x64.png";
	public static final String ICON_CAPABILITY_DEFINITION_128 = "roleDefinition128x128.png";
	
	public static final String ICON_UNKNOWN_TYPE = "unknownType16x16.png";
	
	public static final String ICON_RUNTIME_CONDITION_12 = "runtimeCondition12x12.png";
	public static final String ICON_RUNTIME_CONDITION_16 = "runtimeCondition16x16.png";
	public static final String ICON_RUNTIME_CONDITION_20 = "runtimeCondition20x20.png";
	public static final String ICON_RUNTIME_CONDITION_24 = "runtimeCondition24x24.png";
	
	public static final String ICON_SUCCESS_TRANSITION_12 = "successTransition12x12.png";
	public static final String ICON_SUCCESS_TRANSITION_16 = "successTransition16x16.png";
	public static final String ICON_SUCCESS_TRANSITION_24 = "successTransition24x24.png";
	
	public static final String ICON_SYNCHRONISATION_16 = "synchronisation16x11.png";
	public static final String ICON_SYNCHRONISATION_24 = "synchronisation24x16.png";
	public static final String ICON_SYNCHRONISATION_36 = "synchronisation36x24.png";
	
	public static final String ICON_SYNC_TRANSITION_12 = "syncTransition12x12.png";
	public static final String ICON_SYNC_TRANSITION_16 = "syncTransition16x16.png";
	public static final String ICON_SYNC_TRANSITION_24 = "syncTransition24x24.png";
	
	public static final String ICON_INITIAL_STATE_CONNECTION_12 = "initStateConnection12x12.png";
	public static final String ICON_INITIAL_STATE_CONNECTION_16 = "initStateConnection16x16.png";
	public static final String ICON_INITIAL_STATE_CONNECTION_24 = "initStateConnection24x24.png";
	
	public static final String ICON_MASTER_PLAN_16 = "masterPlan16x16.png";
	public static final String ICON_MASTER_PLAN_24 = "masterPlan24x24.png";
	
	public static final String ICON_FAILURE_TRANSITION_12 = "failureTransition12x12.png";
	public static final String ICON_FAILURE_TRANSITION_16 = "failureTransition16x16.png";
	public static final String ICON_FAILURE_TRANSITION_24 = "failureTransition24x24.png";
	
	public static final String ICON_HIDDEN_16 = "hidden16x16.png";
	
	public static final String ICON_OVERVIEW_16 = "overview16x16.png";
	
	public static final String ICON_ENTRY_POINT_15 = "entryPoint15x15.png";
	public static final String ICON_ENTRY_POINT_16 = "entryPoint16x16.png";
	public static final String ICON_ENTRY_POINT_24 = "entryPoint24x24.png";
	
	public static final String ICON_SUCCESS_POINT_15 = "successPoint15x15.png";
	public static final String ICON_SUCCESS_POINT_16 = "successPoint16x16.png";
	public static final String ICON_SUCCESS_POINT_24 = "successPoint24x24.png";
	
	public static final String ICON_FAILURE_POINT_15 = "failurePoint15x15.png";
	public static final String ICON_FAILURE_POINT_16 = "failurePoint16x16.png";
	public static final String ICON_FAILURE_POINT_24 = "failurePoint24x24.png";
	
	public static final String ICON_STATE_16 = "state16x16.png";
	public static final String ICON_STATE_24 = "state24x24.png";
	public static final String ICON_STATE_50 = "state50x50.png";
	
	public static final String ICON_TASK_16 = "task16x16.png";
	public static final String ICON_TASK_24 = "task24x24.png";
	public static final String ICON_TASK_32 = "task32x32.png";
	public static final String ICON_TASK_64 = "task64x64.png";
	public static final String ICON_TASK_128 = "task128x128.png";

	public static final String ICON_UTILITY_ADD_16 = "utility+16x16.png";
	public static final String ICON_UTILITY_ADD_24 = "task24x24.png";
	public static final String ICON_UTILITY_ADD_32 = "task32x32.png";
	public static final String ICON_UTILITY_ADD_64 = "task64x64.png";
	public static final String ICON_UTILITY_ADD_128 = "utility128x128.png";

	public static final String ICON_UTILITY_REMOVE_16 = "utility-16x16.png";
	public static final String ICON_UTILITY_REMOVE_24 = "task24x24.png";
	public static final String ICON_UTILITY_REMOVE_32 = "task32x32.png";
	public static final String ICON_UTILITY_REMOVE_64 = "task64x64.png";
	public static final String ICON_UTILITY_REMOVE_128 = "utility-128x128.png";
	
	public static final String ICON_TRANSITION_12 = "transition12x12.png";
	public static final String ICON_TRANSITION_16 = "transition16x16.png";
	public static final String ICON_TRANSITION_24 = "transition24x24.png";
	
	public static final String ICON_ZOOM_IN_OUT_16 = "zoom16x16.png";
	public static final String ICON_ZOOM_IN_OUT_24 = "zoom24x24.png";
	public static final String ICON_ARROW_RIGHT_10 = "arrowRight10x10.png";
	public static final String ICON_ARROW_RIGHT_DISABLE_10 = "arrowRightDisable10x10.png";
	public static final String ICON_ARROW_DOWN_10 = "arrowDown10x10.png";
	
	
	public static final String ICON_CURSOR_ZOOM_IN_OUT = "zoominout_source.bmp";
	public static final String ICON_CURSOR_ZOOM_MASK = "zoom_mask.bmp";
	
	public static final String ICON_DELETE_EDIT = "delete_edit.gif";

	

}


