/*******************************************************************************
 * Copyright (c) 2004, 2005 Elias Volanakis and others.
�* All rights reserved. This program and the accompanying materials
�* are made available under the terms of the Eclipse Public License v1.0
�* which accompanies this distribution, and is available at
�* http://www.eclipse.org/legal/epl-v10.html
�*
�* Contributors:
�*����Elias Volanakis - initial API and implementation
�*******************************************************************************/
package de.uni_kassel.vs.cn.planDesigner.ui.util;

import org.eclipse.gef.palette.CombinedTemplateCreationEntry;
import org.eclipse.gef.palette.ConnectionCreationToolEntry;
import org.eclipse.gef.palette.CreationToolEntry;
import org.eclipse.gef.palette.MarqueeToolEntry;
import org.eclipse.gef.palette.PaletteContainer;
import org.eclipse.gef.palette.PaletteDrawer;
import org.eclipse.gef.palette.PaletteRoot;
import org.eclipse.gef.palette.PaletteToolbar;
import org.eclipse.gef.palette.PanningSelectionToolEntry;
import org.eclipse.gef.palette.ToolEntry;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.EntryPointStateDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.SynchedTransitionDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.tool.BehaviourCreationToolEntry;
import de.uni_kassel.vs.cn.planDesigner.ui.tool.PlanCreationToolEntry;
import de.uni_kassel.vs.cn.planDesigner.ui.tool.PlanningProblemCreationToolEntry;
import de.uni_kassel.vs.cn.planDesigner.ui.tool.PlantypeCreationToolEntry;
import de.uni_kassel.vs.cn.planDesigner.ui.tool.ZoomToolEntry;

/**
 * Utility class that can create a GEF Palette for the planmodeller editor.
 */
public final class PlanmodellerEditorPaletteFactory {

	/** Preference ID used to persist the palette location. */
//	private static final String PALETTE_DOCK_LOCATION = "PlanmodellerEditorPaletteFactory.Location";
	/** Preference ID used to persist the palette size. */
//	private static final String PALETTE_SIZE = "PlanmodellerEditorPaletteFactory.Size";
	/** Preference ID used to persist the flyout palette's state. */
//	private static final String PALETTE_STATE = "PlanmodellerEditorPaletteFactory.State";

	private static PlanDesignerActivator PLUGIN = PlanDesignerActivator.getDefault();

	private PlanmodellerEditorPaletteFactory() {
	}

	/**
	 * Creates the drawer for all basic elements.
	 * 
	 * @return
	 */
	private static PaletteContainer createBasicDrawer() {

		PaletteDrawer componentsDrawer = new PaletteDrawer("Basic");
		
		CombinedTemplateCreationEntry component = new PlanCreationToolEntry("Plan", "Create a plan",
				new ModelCreationFactory(AlicaPackage.eINSTANCE.getPlan()), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_PLAN_16), PLUGIN
						.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_PLAN_24));
		componentsDrawer.add(component);

		component = new PlantypeCreationToolEntry("Plantype", "Create a plantype", new ModelCreationFactory(AlicaPackage.eINSTANCE.getPlanType()), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_PLANTYPE_16), PLUGIN.getImageRegistry()
				.getDescriptor(PlanDesignerConstants.ICON_PLANTYPE_24));
		componentsDrawer.add(component);

		component = new PlanningProblemCreationToolEntry("PlanningProblem", "Create a planningproblem", new ModelCreationFactory(
				AlicaPackage.eINSTANCE.getPlanningProblem()), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_PLANNING_PROBLEM_16), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_PLANNING_PROBLEM_24));
		componentsDrawer.add(component);

		component = new CombinedTemplateCreationEntry("State", "Create a state", new ModelCreationFactory(AlicaPackage.eINSTANCE.getState()), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_STATE_16), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_STATE_24));
		componentsDrawer.add(component);

		component = new BehaviourCreationToolEntry("Behaviour", "Create a behaviour", new ModelCreationFactory(AlicaPackage.eINSTANCE.getBehaviour()), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_BEHAVIOUR_16), PLUGIN.getImageRegistry().getDescriptor(
				PlanDesignerConstants.ICON_BEHAVIOUR_24));
		componentsDrawer.add(component);

		component = new CombinedTemplateCreationEntry("Task", "Create a task", new ModelCreationFactory(AlicaPackage.eINSTANCE.getTask()), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_TASK_16), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_TASK_24));
		componentsDrawer.add(component);
		
		component = new CombinedTemplateCreationEntry("Entry point", "Create an entry point", new ModelCreationFactory(
				AlicaPackage.eINSTANCE.getEntryPoint()), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_ENTRY_POINT_16), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_ENTRY_POINT_24));
		componentsDrawer.add(component);

		component = new CombinedTemplateCreationEntry("Success State", "Create a success state", new ModelCreationFactory(
				AlicaPackage.eINSTANCE.getSuccessState()), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_SUCCESS_POINT_16), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_SUCCESS_POINT_24));
		componentsDrawer.add(component);

		component = new CombinedTemplateCreationEntry("Failure State", "Create a failure state", new ModelCreationFactory(
				AlicaPackage.eINSTANCE.getFailureState()), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_FAILURE_POINT_16), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_FAILURE_POINT_24));
		componentsDrawer.add(component);
		
		component = new CombinedTemplateCreationEntry("Synchronisation", "Create an Synchronisation", new ModelCreationFactory(
				AlicaPackage.eINSTANCE.getSynchronisation()), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_SYNCHRONISATION_16), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_SYNCHRONISATION_24));
		componentsDrawer.add(component);

		return componentsDrawer;
	}

	/**
	 * Creates the drawer for all conditions.
	 * 
	 * @return
	 */
	private static PaletteContainer createConditionsDrawer() {
		PaletteDrawer componentsDrawer = new PaletteDrawer("Conditions");

		CombinedTemplateCreationEntry component = new CombinedTemplateCreationEntry("Precondition", "Create a precondition", new ModelCreationFactory(
				AlicaPackage.eINSTANCE.getPreCondition()), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_PRE_CONDITION_16), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_PRE_CONDITION_24));
		componentsDrawer.add(component);

		component = new CombinedTemplateCreationEntry("Runtime condition", "Create a runtime condition", new ModelCreationFactory(
				AlicaPackage.eINSTANCE.getRuntimeCondition()), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_RUNTIME_CONDITION_16), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_RUNTIME_CONDITION_24));
		componentsDrawer.add(component);

		component = new CombinedTemplateCreationEntry("Postcondition", "Create a post condition", new ModelCreationFactory(AlicaPackage.eINSTANCE.getPostCondition()), PLUGIN
				.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_POST_CONDITION_16), PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_POST_CONDITION_24));
		componentsDrawer.add(component);

		return componentsDrawer;
	}

	/**
	 * Creates the drawer for all transitions.
	 * 
	 * @return
	 */
	private static PaletteContainer createTransitionsDrawer() {
		PaletteDrawer componentsDrawer = new PaletteDrawer("Connections");

		CreationToolEntry component = new ConnectionCreationToolEntry("Transition", "Create a transition", 
				new ModelCreationFactory(AlicaPackage.eINSTANCE.getTransition()), 
				PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_TRANSITION_16), 
				PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_TRANSITION_24));
		componentsDrawer.add(component);

		component = new ConnectionCreationToolEntry("Sync Transition", "Sync a transition", 
				new ConnectionCreationFactory(new SynchedTransitionDummyConnection()), 
				PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_SYNC_TRANSITION_16), 
				PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_SYNC_TRANSITION_24));
		componentsDrawer.add(component); 

		component = new ConnectionCreationToolEntry("Init State", "Choose initial state for an entry point", 
				new ConnectionCreationFactory(new EntryPointStateDummyConnection()), 
				PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_INITIAL_STATE_CONNECTION_16), 
				PLUGIN.getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_INITIAL_STATE_CONNECTION_24));
		componentsDrawer.add(component);

		return componentsDrawer;
	}

	/**
	 * Creates the PaletteRoot and adds all palette elements. Use this factory
	 * method to create a new palette for your graphical editor.
	 * 
	 * @return a new PaletteRoot
	 */
	public static PaletteRoot createPalette() {
		PaletteRoot palette = new PaletteRoot();
		palette.add(createToolsGroup(palette));
		palette.add(createBasicDrawer());
		palette.add(createConditionsDrawer());
		palette.add(createTransitionsDrawer());
		return palette;
	}

	/** Create the "Tools" group. */
	private static PaletteContainer createToolsGroup(PaletteRoot palette) {
		PaletteToolbar toolbar = new PaletteToolbar("Tools");

		// Add a selection tool to the group
		ToolEntry tool = new PanningSelectionToolEntry();
		toolbar.add(tool);
		palette.setDefaultEntry(tool);

		// Add a marquee tool to the group
		toolbar.add(new MarqueeToolEntry());

		// Add zoom tools
		toolbar.add(new ZoomToolEntry());

		return toolbar;
	}

}