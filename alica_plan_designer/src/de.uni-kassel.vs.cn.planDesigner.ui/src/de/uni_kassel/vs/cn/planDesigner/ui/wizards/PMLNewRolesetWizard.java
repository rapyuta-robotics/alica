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
package de.uni_kassel.vs.cn.planDesigner.ui.wizards;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import org.eclipse.core.resources.IContainer;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.Path;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.operation.IRunnableWithProgress;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.wizard.Wizard;
import org.eclipse.ui.INewWizard;
import org.eclipse.ui.IWorkbench;
import org.eclipse.ui.IWorkbenchPage;
import org.eclipse.ui.IWorkbenchWizard;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.ide.IDE;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.Role;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleDefinitionSet;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleSet;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLChoosePlanWizardPage;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewRolesetWizardPage;

/**
 * Wizard to generate a new .rset file. 
 */

public class PMLNewRolesetWizard extends Wizard implements INewWizard {
	private PMLNewRolesetWizardPage filePage;
	private ISelection selection;
	
	private PMLTransactionalEditingDomain editingDomain;
	
	private boolean openWhenFinish = true;
	private PMLChoosePlanWizardPage planSelectionPage;
	
	public PMLNewRolesetWizard() {
		super();
		setNeedsProgressMonitor(true);
		setWindowTitle("New Roleset");
		this.editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
				PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
	}
	
	/**
	 * Adding the filePage to the wizard.
	 */

	public void addPages() {
		
		filePage = new PMLNewRolesetWizardPage(selection);
		planSelectionPage = new PMLChoosePlanWizardPage();
		if(!filePage.checkIfRoleDefinitionPathExists())
		{
			if(filePage.createRoleDefinitionFile())
			{
				if(filePage.roleDefinitionFile())
				{
					addPage(filePage);
					addPage(planSelectionPage);	
				}
			}
		}
	}
	
	/**
	 * This method is called when 'Finish' button is pressed in
	 * the wizard. We will create an operation and run it
	 * using wizard as execution context.
	 */
	public boolean performFinish() {
		final String containerName = filePage.getContainerName();
		final String fileName = PlanEditorUtils.removeFileExtension(filePage.getFileName()) + ".rset";
		final IResource planFile = planSelectionPage.getSelectedResource();
		
		IRunnableWithProgress op = new IRunnableWithProgress() {
			public void run(IProgressMonitor monitor) throws InvocationTargetException {
				try {
					doFinish(containerName, fileName, planFile, monitor);
				} catch (CoreException e) {
					throw new InvocationTargetException(e);
				} finally {
					monitor.done();
				}
			}
		};
		try {
			getContainer().run(true, false, op);
		} catch (InterruptedException e) {
			return false;
		} catch (InvocationTargetException e) {
			Throwable realException = e.getTargetException();
			MessageDialog.openError(getShell(), "Error", realException.getMessage());
			return false;
		}
		return true;
	}
	
	/**
	 * The worker method. It will find the container, create the
	 * file if missing or just replace its contents, and open
	 * the editor on the newly created file.
	 * @param planFile 
	 */

	private void doFinish(String containerName, String fileName, IResource planResource, IProgressMonitor monitor)
																	throws CoreException {
		monitor.beginTask("Creating " + fileName, openWhenFinish ? 2 : 1);
		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		IResource containerResource = root.findMember(new Path(containerName));
		
		if (!containerResource.exists() || !(containerResource instanceof IContainer)) {
			throwCoreException("Container \"" + containerName + "\" does not exist.");
		}
		if (!planResource.exists() || !(planResource instanceof IFile)) {
			throwCoreException("File \"" + planResource + "\" does not exist.");
		}
		
		IContainer container = (IContainer) containerResource;
		final IFile roleSetFile = container.getFile(new Path(fileName));
		final IFile planFile = (IFile)planResource;
	
		if (roleSetFile.exists()) {
			System.err.println("Overwriting existing roleset!!!");
		} else {
			// Create an empty file
//			roleSetFile.create(null, true, monitor);
//			 Init the file with the given plan
			initFileWithStandardTemplate(roleSetFile, planFile, monitor);
		} 

		monitor.worked(1);
				
		if(openWhenFinish){
			monitor.setTaskName("Opening file for editing...");
			getShell().getDisplay().asyncExec(new Runnable() {
				public void run() {
					IWorkbenchPage page =
						PlatformUI.getWorkbench().getActiveWorkbenchWindow().getActivePage();
					try {
						IDE.openEditor(page, roleSetFile, true);
					} catch (PartInitException e) {
					}
				}
			});
			monitor.worked(1);
		}
	}
	
	private void initFileWithStandardTemplate(final IFile roleSetFile, IFile planFile, IProgressMonitor monitor){
		final ResourceSet globalResourceSet = editingDomain.getResourceSet();
		
		// Try to find the roledefinition file
		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		IFile roledefinitionFile = root.getFile(CommonUtils.getRoleDefinitionPath());
		
		monitor.subTask("Loading global role definitions");
		
		// Load the roledefintion into the global resourceset
		RoleDefinitionSet rdefset = (RoleDefinitionSet)globalResourceSet.getResource(URI.createPlatformResourceURI(
				roledefinitionFile.getFullPath().toString(),true), true).getContents().get(0);
		
		monitor.worked(1);
		
		monitor.subTask("Reading plan");
		
		// Get the plan
		final Plan plan = (Plan)globalResourceSet.getResource(URI.createPlatformResourceURI(planFile.getFullPath().toString(),
				true), true).getContents().get(0);
		
		monitor.worked(1);
		
		monitor.subTask((roleSetFile.exists() ? "Loading " : "Creating ") +"roleset file");
		
		// Create a resource for the roleset
		final Resource roleSetResource = globalResourceSet.createResource(
				URI.createPlatformResourceURI(roleSetFile.getFullPath().toOSString(), true));
		
		monitor.worked(1);
		
		monitor.subTask("Collecting tasks");
		
		// Trigger to load all dependent resources
//		loadDescendingPlans(plan);
//		EcoreUtil.resolveAll(plan);
		final Set<Task> allTasks = getTasks(plan, new ArrayList<Plan>());
		
		monitor.worked(1);
		
		monitor.subTask("Generation default mappings");
		
		// Create the roleset
		final RoleSet roleSet = AlicaFactory.eINSTANCE.createRoleSet();
		roleSet.setUsableWithPlanID(plan.getId());
		roleSet.setName(roleSetFile.getName());
		
		CompoundCommand assignTasksCompound = new CompoundCommand("Assigne default task priorities");
		// Assign all tasks for each role
		for(final Role r : rdefset.getRoles()){
			assignTasksCompound.append(new RecordingCommand(editingDomain){
				@Override
				protected void doExecute() {
					roleSet.getMappings().add(assignTasks(allTasks, r));	
				}
			});
		}
		
		CompoundCommand allCommands = new CompoundCommand(0);
		allCommands.append(assignTasksCompound);
		allCommands.append(new RecordingCommand(editingDomain){
			@Override
			protected void doExecute() {
				roleSetResource.getContents().add(roleSet);
				
				try {
					roleSetResource.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		});
		
		editingDomain.getCommandStack().execute(allCommands);
	}
	
	private Set<Task> getTasks(Plan p, Collection<Plan> visitedPlans){
		Set<Task> tasks = new HashSet<Task>();

		// Get the tasks which are attached to 
		// the entrypoints
		for(EntryPoint ep : p.getEntryPoints()){
			tasks.add(ep.getTask());
		}
		
		// Add all tasks which can be reached from the plan
		for(State s : p.getStates()){
			for(AbstractPlan absPlan : s.getPlans()){
				if(absPlan.eClass() == AlicaPackage.eINSTANCE.getPlan())
				{
					if(!visitedPlans.contains(absPlan))
					{	
						visitedPlans.add((Plan)absPlan);
						tasks.addAll(getTasks((Plan)absPlan, visitedPlans));
					}
					
				}
			}
		}
		return tasks;
	}
	
	/**
	 * Creates a new RoleTaskassginement and associates it with the given role.
	 * After that it creates taskpriorities for each task.
	 * @param rSet
	 * @param r
	 * @return
	 */
	public static RoleTaskMapping assignTasks(Set<Task> tasks, Role r){
		RoleTaskMapping rtass = AlicaFactory.eINSTANCE.createRoleTaskMapping();
		rtass.setRole(r);
		
		for(Task t : tasks)
			rtass.getTaskPriorities().put(t.getId(), 0.5);
		
		return rtass;
	}

	private void throwCoreException(String message) throws CoreException {
		IStatus status =
			new Status(IStatus.ERROR, "de.uni_kassel.vs.cn.planDesigner.ui", IStatus.OK, message, null);
		throw new CoreException(status);
	}

	/**
	 * We will accept the selection in the workbench to see if
	 * we can initialize from it.
	 * @see IWorkbenchWizard#init(IWorkbench, IStructuredSelection)
	 */
	public void init(IWorkbench workbench, IStructuredSelection selection) {
		this.selection = selection;
		
		// TODO: Shut down the wizard correctly if that fails
//		// Check if the roleDefinitionFile can be found
//		if(CommonUtils.getRoleDefinitionFile() == null){
//			Display.getDefault().asyncExec(new Runnable(){
//				@Override
//				public void run() {
//					MessageDialog.openError(
//							getShell(), 
//							"Role definition file not found", 
//							"The role definition file can not be found. Either you didn't " +
//							"specify roles or the path to that file is wrong. Please check " +
//							"the preferences!");
//				}
//			});
//			performCancel();
//		}
	}

	public void setOpenWhenFinish(boolean openWhenFinish) {
		this.openWhenFinish = openWhenFinish;
	}
}