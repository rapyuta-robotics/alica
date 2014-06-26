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
package de.uni_kassel.vs.cn.planDesigner.ui.tool;

import java.util.Collections;

import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.commands.UnexecutableCommand;
import org.eclipse.gef.tools.CreationTool;
import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.wizard.WizardDialog;
import org.eclipse.osgi.internal.resolver.ComputeNodeOrder;
import org.eclipse.swt.graphics.Cursor;
import org.eclipse.swt.graphics.Point;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CommandWrap4EMF;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.ExecuteAndRefreshPlanCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.RemoveReadOnlyCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.StateEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLNewBehaviourWizard;


public class BehaviourCreationTool extends CreationTool {
	
	@Override
	protected void performCreation(int button) {
		EditPart target = getTargetEditPart();
		// If target isn't a state we return
		if(target == null || !(target.getModel() instanceof State))
			return;
		
		final EditPartViewer viewer = getCurrentViewer();

		Behaviour createdBehaviour = (Behaviour)getCreateRequest().getNewObject();
		
		// Create a NewPlan wizard and initialize it with the created plan
		PMLNewBehaviourWizard wiz = new PMLNewBehaviourWizard(createdBehaviour);
		
		WizardDialog dialog = new WizardDialog(viewer.getControl().getShell(), wiz){
			@Override
			protected Point getInitialSize() {
				return new Point(600,400);
			}
		};
		
		dialog.setBlockOnOpen(true);
		if(dialog.open() == Dialog.OK){
			// Get the created configuration
			final BehaviourConfiguration config = wiz.getCreatedConfiguration();
			if(config != null){
				final PlanEditor editor = PlanEditorUtils.getPlanEditor(target);
				PMLTransactionalEditingDomain editingDomain = editor.getEditingDomain();
				
				CompoundCommand compound = new CompoundCommand("ADD BEHAVIOR");
					
				org.eclipse.emf.common.command.Command createChildCommandFirst = null;
				org.eclipse.emf.common.command.Command createChildCommandSecond = null;
				//This doesnt work unexecutable commands for one reason that I didnt know
//				compound.append(CreateChildCommand.create(
//						editingDomain,
//						createdBehaviour, 
//						new CommandParameter(createdBehaviour, AlicaPackage.eINSTANCE.getBehaviour_Configurations(),config), 
//						Collections.EMPTY_LIST));
				createChildCommandFirst = CreateChildCommand.create(
						editingDomain, 
						createdBehaviour, 
						new CommandParameter(createdBehaviour, AlicaPackage.eINSTANCE.getBehaviour_Configurations(),config), 
						Collections.emptyList());
				
				//This doesnt work unexecutable commands for one reason that I didnt know
				// Add the new Configuration to the state
//				compound.append(CreateChildCommand.create(
//						editingDomain, 
//						target.getModel(), 
//						new CommandParameter(target.getModel(),AlicaPackage.eINSTANCE.getState_Plans(), config),
//						Collections.EMPTY_LIST));
				
				createChildCommandSecond = CreateChildCommand.create(
						editingDomain, 
						target.getModel(), 
						new CommandParameter(target.getModel(),AlicaPackage.eINSTANCE.getState_Plans(),config), 
						Collections.emptyList());
				
				// Expand the state
				compound.append(SetCommand.create(
						editingDomain, 
						editor.getUIExtension((EObject)target.getModel(), true), 
						PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Collapsed(), 
						Boolean.FALSE));
				
				// Add a command which selects the new configuration
				compound.append(new RecordingCommand(editingDomain){
					@Override
					protected void doExecute() {
						EditPart part = (EditPart)viewer.getEditPartRegistry().get(config);
						if(part != null){
							// Activate the editor
							editor.getSite().getPage().activate(editor);
							viewer.flush();
							viewer.select(part);
						}
					}
				});
				editingDomain.getCommandStack().execute(createChildCommandFirst);
				editingDomain.getCommandStack().execute(createChildCommandSecond);
				editingDomain.getCommandStack().execute(compound);

			
		
			}
		}else{
			// Undo the command
//			getCurrentViewer().getEditDomain().getCommandStack().undo();
		}
		
	}
	
	/**
	 * Overridden to enable the tool when moving over a state
	 */
	@Override
	protected Cursor calculateCursor() {
		if(getTargetEditPart() instanceof StateEditPart)
			return getDefaultCursor();
		else
			return getDisabledCursor();
	}
	
	/**
	 * Overridden to not bother editParts with creation of commands. This was done because
	 * the command doesn't reuse the command's created by the editParts. It builds
	 * it's ownes ones.
	 */
	@Override
	protected Command getCommand() {
		return UnexecutableCommand.INSTANCE;
	}
}
