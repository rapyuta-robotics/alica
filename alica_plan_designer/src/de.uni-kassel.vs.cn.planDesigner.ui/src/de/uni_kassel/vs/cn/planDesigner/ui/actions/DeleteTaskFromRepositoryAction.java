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
package de.uni_kassel.vs.cn.planDesigner.ui.actions;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.emf.ecore.util.EcoreUtil.UsageCrossReferencer;
import org.eclipse.emf.edit.command.DeleteCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.RunnableWithResult;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.action.Action;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.StructuredViewer;
import org.eclipse.jface.window.Window;
import org.eclipse.swt.widgets.Shell;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.RemoveReadOnlyCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.TaskUsageDialog;

public class DeleteTaskFromRepositoryAction extends Action {
	
	private static final String msg = "The task you are trying to delete is attached to one " +
	"or more entry points. If you proceed, all occurences will be replaced by " +
	"a default task!";
	
	private StructuredViewer viewer;

	public DeleteTaskFromRepositoryAction(StructuredViewer viewer){
		this.viewer = viewer;
	}
	
	@Override
	public void run() {
		ISelection selection = viewer.getSelection();
		if(selection != null && shouldReallyDelete()){
			PMLTransactionalEditingDomain domain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
			
			boolean saveNeeded = false;
			
			Task taskToDelete = (Task)((IStructuredSelection)selection).getFirstElement();
			
			PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
			// Check if the task is associated to any entryPoints
			List<EntryPoint> entryPoints = new ArrayList<EntryPoint>();
			Map<EObject, Collection<EStructuralFeature.Setting>> usages = UsageCrossReferencer.findAll(Collections.singleton(taskToDelete), editingDomain.getResourceSet());
//			System.out.println("Task: " +taskToDelete);
			for(EObject obj : usages.keySet())
			{
//				System.out.println("\tEObject: " +obj);
				for(EStructuralFeature.Setting setting : usages.get(obj))
				{
//					System.out.println("\t\tSetting: " +setting.getEObject());
					if(setting.getEObject() instanceof EntryPoint)
					{
						entryPoints.add((EntryPoint)setting.getEObject());
					}
				}
			}
			CompoundCommand cmp = new CompoundCommand();
			if(entryPoints.size() > 0){
				// This task is attach to entryPoints. Tell
				// The user, that all occurences of this
				// Task will be replaced with the default task
				// from the TaskRepository
				Set<Plan> affectedPlans = CommonUtils.getAffectedPlans(entryPoints);
				if(shouldProceed(affectedPlans)){
					// Replace every occurrence of the taskToDelete
					// with the default task
					
					Task defaultTask = CommonUtils.getDefaultTaskFromTaskRepository(domain);
					for(EntryPoint ep : entryPoints){
						cmp.append(SetCommand.create(
								domain, 
								ep, 
								AlicaPackage.eINSTANCE.getEntryPoint_Task(), 
								defaultTask));
					}
					for(final Plan p : affectedPlans){
						cmp.append(new RecordingCommand(domain){
							@Override
							protected void doExecute() {
								try {
									p.eResource().save(
											AlicaSerializationHelper.getInstance().getLoadSaveOptions());
								} catch (IOException e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}
							}
						});
					}
					saveNeeded = true;
				}else
					return;
			}
			
			// Enable write permission
			editingDomain.getCommandStack().execute(new RemoveReadOnlyCommand(editingDomain));
			
			// Delete the task
			cmp.append(
					DeleteCommand.create(
							domain, 
							Collections.singletonList(taskToDelete)));
			saveNeeded = true;
			
			domain.getCommandStack().execute(cmp.unwrap());
			
			if(saveNeeded){
				// Save the task repository
				try {
					CommonUtils.getTaskRepositoryResource(domain).save(
							AlicaSerializationHelper.getInstance().getLoadSaveOptions());
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}

	private boolean shouldReallyDelete() {
		final Shell shell = viewer.getControl().getShell();
		RunnableWithResult<Boolean> run = new RunnableWithResult.Impl<Boolean>(){
			public void run() {
				if(MessageDialog.openQuestion(
						shell, 
						"Confirm delete", 
						"Do you really want to delete the selected items?"))
					setResult(true);
				else
					setResult(false);
			}
		};
		
		shell.getDisplay().syncExec(run);
		return run.getResult();
	}

	private boolean shouldProceed(final Set<Plan> affectedPlans) {
		final Shell shell = viewer.getControl().getShell();
		RunnableWithResult<Boolean> run = new RunnableWithResult.Impl<Boolean>(){
			public void run() {
				TaskUsageDialog dia = new TaskUsageDialog(shell, affectedPlans, "Confirm delete", msg, MessageDialog.WARNING);
				if(dia.open() == Window.OK)
					setResult(true);
				else
					setResult(false);
			}
		};
		
		shell.getDisplay().syncExec(run);
		return run.getResult();
	}

	

}
