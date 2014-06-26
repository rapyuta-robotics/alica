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
import org.eclipse.swt.graphics.Cursor;
import org.eclipse.swt.graphics.Point;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.StateEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLNewPlantypeWizard;

public class PlanTypeCreationTool extends CreationTool {
	
	@Override
	protected void performCreation(int button) {
		EditPart target = getTargetEditPart();
		// If target isn't a state we return
		if(target == null || !(target.getModel() instanceof State))
			return;
		
		final EditPartViewer viewer = getCurrentViewer();
		
		PlanType createdPlanType = (PlanType)getCreateRequest().getNewObject();
		
		// Create a NewPlan wizard and initialize it with the created plan
		PMLNewPlantypeWizard wiz = new PMLNewPlantypeWizard(createdPlanType);
		
		WizardDialog dialog = new WizardDialog(viewer.getControl().getShell(), wiz){
			@Override
			protected Point getInitialSize() {
				return new Point(600,400);
			}
		};
		
		dialog.setBlockOnOpen(true);
		if(dialog.open() == Dialog.OK){
			// Get the created configuration
			final PlanType planType = wiz.getCreatedPlanType();
			
			final PlanEditor editor = PlanEditorUtils.getPlanEditor(target);
			PMLTransactionalEditingDomain editingDomain = editor.getEditingDomain();
			
			CompoundCommand compound = new CompoundCommand(0);
			org.eclipse.emf.common.command.Command createChildCommandFirst = null;
			
			//This doesnt work unexecutable commands for one reason that I didnt know
			// Add the new planType to the state
//			compound.append(CreateChildCommand.create(
//					editingDomain, 
//					target.getModel(), 
//					new CommandParameter(null,null,planType),
//					Collections.EMPTY_LIST));
			createChildCommandFirst = CreateChildCommand.create(
					editingDomain, 
					target.getModel(), 
					new CommandParameter(target.getModel(), AlicaPackage.eINSTANCE.getState_Plans(), planType), 
					Collections.emptyList());
			
			// Expand the state
			compound.append(SetCommand.create(
					editingDomain, 
					editor.getUIExtension((EObject)target.getModel(), true), 
					PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Collapsed(), 
					Boolean.FALSE));
			
			// Add a command which selects the new plantype
			compound.append(new RecordingCommand(editingDomain){
				@Override
				protected void doExecute() {
					EditPart part = (EditPart)viewer.getEditPartRegistry().get(planType);
					if(part != null){
						// Activate the editor
						editor.getSite().getPage().activate(editor);
						viewer.flush();
						viewer.select(part);
					}
				}
			});
			
			// Execute the whole thing
			editingDomain.getCommandStack().execute(createChildCommandFirst);
			editingDomain.getCommandStack().execute(compound);
			
			// Create command which adds the configuration to the targeteditpart
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
