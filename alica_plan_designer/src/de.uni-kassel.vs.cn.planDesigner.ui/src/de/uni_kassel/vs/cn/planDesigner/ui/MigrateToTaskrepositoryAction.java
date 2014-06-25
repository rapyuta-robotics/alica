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
package de.uni_kassel.vs.cn.planDesigner.ui;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.RunnableWithResult;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.action.IAction;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.ui.IObjectActionDelegate;
import org.eclipse.ui.IWorkbenchPart;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskRepository;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;;

public class MigrateToTaskrepositoryAction implements IObjectActionDelegate {
	
	private IWorkbenchPart targetPart;
	private ISelection selection;

	public MigrateToTaskrepositoryAction() {
	}

	public void setActivePart(IAction action, IWorkbenchPart targetPart) {
		this.targetPart = targetPart;
	}

	public void run(IAction action) {
		// Get the workspace root
		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		
		// Get the domain
		PMLTransactionalEditingDomain domain =  (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
				PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		
		// Get the global resourceset
		ResourceSet gloablResourceSet = domain.getResourceSet();
		
		// Get the filehandle to the taskrepository
		IFile taskRepositoryFile = CommonUtils.getTaskRepositoryFile();
		
		final Resource taskRepositoryResource;
		
		if(taskRepositoryFile.exists()){
			if(overrideTaskRepository()){
				taskRepositoryResource = gloablResourceSet.getResource(URI.createPlatformResourceURI(taskRepositoryFile.getFullPath().toString(),
						true), true);
			}else
				return;
		}else
			taskRepositoryResource = gloablResourceSet.createResource(URI.createPlatformResourceURI(taskRepositoryFile.getFullPath().toString(),
					true));
		
		CompoundCommand createTaskRepositoryCommands = new CompoundCommand("Create taskrepository");
		
		final TaskRepository repo = AlicaFactory.eINSTANCE.createTaskRepository();
		
		// Collect all *.pml files
		Set<IFile> allPlanFiles = CommonUtils.collectAllFilesWithExtension("pml");
		
		// Load all plans into the global resource set
		List<Resource> loadedPlans = new ArrayList<Resource>();
		for(IFile planFile : allPlanFiles){
			loadedPlans.add(domain.load(planFile));
		}
		
		// For each loaded plan...
		for(Resource r : loadedPlans){
			Plan plan = (Plan)r.getContents().get(0);
			// ... look at the entry points
			for(final EntryPoint ep : plan.getEntryPoints()){
				// ... and create a command which adds the 
				// task of that entrypoint to the taskrepository
				createTaskRepositoryCommands.append(new RecordingCommand(domain){
					@Override
					protected void doExecute() {
						repo.getTasks().add(ep.getTask());
					}
				});
			}
		}
		
		createTaskRepositoryCommands.append(new RecordingCommand(domain){
			@Override
			protected void doExecute() {
				// Clear any possible contents
				taskRepositoryResource.getContents().clear();
			}
		});
		
		// Add a command to create a new repository
		createTaskRepositoryCommands.append(new RecordingCommand(domain){
			@Override
			protected void doExecute() {
				taskRepositoryResource.getContents().add(repo);
			}
		});
		
		// Execute the commands
		domain.getCommandStack().execute(createTaskRepositoryCommands);
		
		try {
			// Save the task repository
			taskRepositoryResource.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
			
			// Save each loaded plan
			for(Resource r : loadedPlans)
				r.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		showFinishMessage();

	}

	private void showFinishMessage() {
		final Shell shell = targetPart.getSite().getShell();
		shell.getDisplay().syncExec(new Runnable(){
			public void run() {
				MessageDialog.openInformation(
						shell, 
						"Migrating successful", 
						"Migration to Taskrepository system successfully finished!");
			}
		});
	}

	private boolean overrideTaskRepository() {
		final Shell shell = targetPart.getSite().getShell();
		
		RunnableWithResult<Boolean> question = new RunnableWithResult.Impl<Boolean>(){
			public void run() {
				setResult(MessageDialog.openQuestion(shell, 
						"Overwrite Taskrepository", 
						"There is already a taskrepository in the workspace.\n\n" +
						"Do you really want to overwrite this repository?"));
			}
		};
		
		shell.getDisplay().syncExec(question);
		
		return question.getResult();
	}

	public void selectionChanged(IAction action, ISelection selection) {
		this.selection = selection;
	}

}
