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
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.Request;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.commands.UnexecutableCommand;
import org.eclipse.gef.editpolicies.LayoutEditPolicy;
import org.eclipse.gef.editpolicies.NonResizableEditPolicy;
import org.eclipse.gef.requests.CreateRequest;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskRepository;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.IModelExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CommandWrap4EMF;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.EntryPointEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public class EntryPointLayoutEditPolicy extends LayoutEditPolicy {

	@Override
	protected EditPolicy createChildEditPolicy(EditPart child) {
		return new NonResizableEditPolicy();
	}

	@Override
	protected Command getCreateCommand(CreateRequest request) {
		Command cmd = UnexecutableCommand.INSTANCE;
		final EObject newChild = (EObject)request.getNewObject();
		
		// We only know how to build children in EObject containers
		if(getHost().getModel() instanceof EObject){
			EObject parent = (EObject)getHost().getModel();
		
			IModelExclusionAdapter exclusionAdapter = (IModelExclusionAdapter)getHost().getAdapter(IModelExclusionAdapter.class);
			Set<String> exclusionSet = null;
			
			if(exclusionAdapter != null)
				exclusionSet = exclusionAdapter.getExclusionClasses();
			
			// Only create the command if the child isn't in the exclusion set
			if(exclusionSet == null || (exclusionSet != null && !exclusionSet.contains(newChild.eClass().getName()))){
				PMLTransactionalEditingDomain editingDomain =  (PMLTransactionalEditingDomain)TransactionUtil.getEditingDomain(parent);
				
				CompoundCommand cmp = new CompoundCommand();
				
				if(newChild.eClass() == AlicaPackage.eINSTANCE.getTask()){
					// In case the user wants to create a task, we have to 
					// ensure, that the task is attached to the task repository
					PlanEditor editor = PlanEditorUtils.getPlanEditor(getHost());
					final TaskRepository repository = CommonUtils.getTaskRepository(editingDomain,true);
					
					cmp.append(new RecordingCommand(editor.getEditingDomain()){
						@Override
						protected void doExecute() {
							repository.getTasks().add((Task)newChild);
						}
					});
					
					cmp.append(SetCommand.create(editor.getEditingDomain(), parent, AlicaPackage.eINSTANCE.getEntryPoint_Task(), newChild));
				}else{
					// Add the creatChild command
					cmp.append(CreateChildCommand.create(
							editingDomain, 
							parent, 
							new CommandParameter(null,null,newChild), 
							Collections.emptyList()));
				}
				
				cmd = new CommandWrap4EMF(cmp.unwrap());
			}
		}
		
		return cmd;
	}

	@Override
	protected Command getMoveChildrenCommand(Request request) {
		return null;
	}
	

}
