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
package de.uni_kassel.vs.cn.planDesigner.ui.policies;

import java.util.Collections;
import java.util.Set;

import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.util.TransactionUtil;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.Request;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.commands.UnexecutableCommand;
import org.eclipse.gef.editpolicies.GraphicalEditPolicy;
import org.eclipse.gef.requests.CreateRequest;
import org.eclipse.gef.requests.GroupRequest;
import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.wizard.WizardDialog;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.impl.TaskImpl;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.IModelExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CommandWrap4EMF;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.RemoveReadOnlyCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLNewBehaviourConfigurationWizard;

public class DragDropWithoutOrphan extends GraphicalEditPolicy  {

	@Override
	public boolean understandsRequest(Request req) {
		return req.getType().equals(RequestConstants.REQ_DROP) || 
		req.getType().equals(RequestConstants.REQ_DRAG) || 
		req.getType().equals(REQ_ADD);
	}
	
	/**
	 * @see org.eclipse.gef.EditPolicy#getTargetEditPart(Request)
	 */
	public EditPart getTargetEditPart(final Request request) {
		// At this point we could use better strategy to signal the user if
		// a certain object could be dropped at the host. The problem is, that
		// in case of the TemplatTranser, the TemplateTransfers template is only set
		// if the user performs the actual drop, so we don't know anything about the 
		// object being dropped. For now we just return the host if this editPolicy understands
		// the request.
		return understandsRequest(request) ? getHost() : null;
	}
	
	@Override
	public Command getCommand(Request request) {
		if(request.getType().equals(RequestConstants.REQ_DROP)){
			return getDropCommand((CreateRequest)request);
		} else if(request.getType().equals(RequestConstants.REQ_DRAG)){
			return getDragCommand((CreateRequest)request);
		} else if(request.getType().equals(REQ_ADD)) {
			return getAddChildrenCommand(request);
		}else if(request.getType().equals(REQ_ORPHAN_CHILDREN)){
			return getOrphanChildrenCommand((GroupRequest) request);
		}

		return null;
	}
	public Command getOrphanChildrenCommand(GroupRequest request) {
		return null;
	}
	
	public Command getAddChildrenCommand(Request request) {
		Command wrap = UnexecutableCommand.INSTANCE;
		return wrap;
	}
	/**
	 * Get a command which indicates if the requested drag operation can
	 * be executed. That's useful to just test if potentially the user could
	 * drop the item on the host or not.
	 * @param req
	 * @return
	 */
	protected Command getDragCommand(CreateRequest req){
		return getDropCommand(req);
		
	}
	
	/**
	 * Indicates that the user want to drop something on the host.
	 * @param req
	 * @return
	 */
	protected Command getDropCommand(CreateRequest req){
		Command cmd = UnexecutableCommand.INSTANCE;
		// Check what the user wants to build
		if(req.getNewObjectType().equals(AlicaPackage.eINSTANCE.getBehaviour())){
			// We have to assist the user in creating a new BehaviourConfiguration
			final Behaviour behaviour = (Behaviour)req.getNewObject();
			
			// Populate a BehaviourConfigurationWizard
			PMLNewBehaviourConfigurationWizard wiz = new PMLNewBehaviourConfigurationWizard(behaviour);
			// We do not want the plan to get opened
//			wiz.setOpenWhenFinish(false);
			
			final EditPartViewer viewer = getHost().getViewer();
			WizardDialog dialog = new WizardDialog(viewer.getControl().getShell(), wiz);
			
			dialog.setBlockOnOpen(true);
			if(dialog.open() == Dialog.OK){
				final BehaviourConfiguration config = wiz.getCreatedConfiguration();
				if(config != null){
					final PlanEditor editor = PlanEditorUtils.getPlanEditor(getHost());
					final PMLTransactionalEditingDomain editingDomain = editor.getEditingDomain();
					
					// Enable write permission
					editingDomain.getCommandStack().execute(new RemoveReadOnlyCommand(editingDomain));
					
					// Create a command which adds the new configuration to the behaviour
					CompoundCommand compound = new CompoundCommand();
					compound.append(CreateChildCommand.create(
							editingDomain,
							behaviour, 
							new CommandParameter(behaviour,AlicaPackage.eINSTANCE.getBehaviour_Configurations(),config), 
							Collections.EMPTY_LIST));
					// Create a command which adds the configuration to the abstractplan
					compound.append(CreateChildCommand.create(
							editingDomain,
							getHost().getModel(),
							new CommandParameter(getHost().getModel(),AlicaPackage.eINSTANCE.getState_Plans(),config), 
							Collections.EMPTY_LIST));
					// Expand the state
					compound.append(SetCommand.create(
							editingDomain, 
							editor.getUIExtension((EObject)getHost().getModel(), true), 
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
					
					cmd = new CommandWrap4EMF(compound);
				}
				
				
			}
		}else{
			// Just return the command which will build the child
			cmd = getCreateDropCommand(req);
		}
		
		return cmd;
	}
	
	/**
	 * Creates a command which can build the requested new object.
	 * @param req
	 * @return
	 */
	protected Command getCreateDropCommand(CreateRequest req){
		// Copied from PMLFlowLayoutEditPolicy...
		
		// TODO: Remove this - only for debugging purposis. This should not be happen!
		if(req.getNewObject() == null){
			System.err.println("Creation Factory doesn't return a new Object!");
			return null;
		}
		
		Command cmd = UnexecutableCommand.INSTANCE;
		
		final EObject newChild = (EObject)req.getNewObject();
		
		// We only know how to build children in EObject containers
		if(getHost().getModel() instanceof EObject){
			final EObject parent = (EObject)getHost().getModel();
			
			IModelExclusionAdapter exclusionAdapter = (IModelExclusionAdapter)getHost().getAdapter(IModelExclusionAdapter.class);
			Set<String> exclusionSet = null;
			if(exclusionAdapter != null)
				exclusionSet = exclusionAdapter.getExclusionClasses();
			
			// Only create the command if the child isn't in the exclusion set
			if(exclusionSet == null || (exclusionSet != null && !exclusionSet.contains(newChild.eClass().getName()))){
				final PMLTransactionalEditingDomain editingDomain =  (PMLTransactionalEditingDomain)TransactionUtil.getEditingDomain(parent);
				
				editingDomain.getCommandStack().execute(new RemoveReadOnlyCommand(editingDomain));
				
				org.eclipse.emf.common.command.Command createChildCommand = null;
				// This is a hack, cause CreateChildCommand returns an unexecutable
				// command, if the owners feature is single-valued.
				if(newChild instanceof Task){
					createChildCommand = SetCommand.create(
							editingDomain, 
							parent, 
							AlicaPackage.eINSTANCE.getEntryPoint_Task(), 
							newChild);
				}else if (newChild instanceof AbstractPlan){
					createChildCommand = CreateChildCommand.create(
						editingDomain, 
						parent, 
						new CommandParameter(parent,AlicaPackage.eINSTANCE.getState_Plans(),newChild), 
						Collections.emptyList());
				}else {
					System.err.println("UNHANDELD DRAG AND DROP");
				}
				cmd = new CommandWrap4EMF(createChildCommand);
			}
		}
		
		return cmd;
	}
}
