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
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;

import org.eclipse.core.commands.operations.OperationStatus;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.IPath;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.MultiStatus;
import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.jface.action.IAction;
import org.eclipse.jface.dialogs.ErrorDialog;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.dialogs.ProgressMonitorDialog;
import org.eclipse.jface.operation.IRunnableWithProgress;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.ui.IObjectActionDelegate;
import org.eclipse.ui.IWorkbenchPart;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.CapValue;
import de.uni_kassel.vs.cn.planDesigner.alica.Characteristic;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.Role;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleDefinitionSet;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleSet;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;


public class GenerateRolesetAction implements IObjectActionDelegate {

	private ISelection selection;
	
	private IWorkbenchPart targetPart;
	
	private Shell shell;
	
	private boolean shouldGenerate;

	public void setActivePart(IAction action, IWorkbenchPart targetPart) {
		this.targetPart = targetPart;
		this.shell = targetPart.getSite().getShell();
	}

	public void run(IAction action) {
		final MultiStatus status = new MultiStatus(PlanDesignerActivator.PLUGIN_ID, 90 , "Errors during roleset generation", null);
		
		// Define the operation
		IRunnableWithProgress op = new IRunnableWithProgress(){
			public void run(IProgressMonitor monitor)
					throws InvocationTargetException, InterruptedException {
				generateRoleSet(monitor, status);				
			}
		};
		
		// Run the update within a progressMonitor
		try {
			new ProgressMonitorDialog(shell).run(true, false, op);
		} catch (InvocationTargetException e) {
			status.add(new OperationStatus(
					OperationStatus.ERROR,
					PlanDesignerActivator.PLUGIN_ID, 94,
					"Application faild with error", e));
		} catch (InterruptedException e) {
			status.add(new OperationStatus(
					OperationStatus.CANCEL,
					PlanDesignerActivator.PLUGIN_ID, 95,
					"Roleset generation cancelled", null));
		}
		
		if(!status.isOK()){
			ErrorDialog dialog = new ErrorDialog(shell,
					"Generate Roleset", "Error during roleset generation", status, OperationStatus.ERROR | OperationStatus.WARNING);
			dialog.open();
		}else if(shouldGenerate){
			shell.getDisplay().syncExec(new Runnable(){
				public void run() {
					MessageDialog.openInformation(
							shell, 
							"Generate Roleset", 
							"Generation of the roleset was successfull.");
				}
			});
		}
		
	}
	
	private void generateRoleSet(IProgressMonitor monitor, MultiStatus status) {
		monitor.beginTask("Generating Roleset", 6);
		
		IFile file = (IFile)((IStructuredSelection)selection).getFirstElement();
		
		// Try to find the roledefinition file
		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		IFile roledefinitionFile = root.getFile(CommonUtils.getRoleDefinitionPath());
		
		if(!roledefinitionFile.exists()){
			showRoleDefinitionError();
			monitor.done();
			return;
		}
		
		monitor.subTask("Loading role definitions");
		
		// Create a local resourceset
		ResourceSet localResourceSet = CommonUtils.getAlicaResourceSet();
		
		// Load the roledefintion into the global resourceset
		RoleDefinitionSet rdefset = (RoleDefinitionSet)localResourceSet.getResource(URI.createPlatformResourceURI(roledefinitionFile.getFullPath().toString(),
				true), true).getContents().get(0);
		
		// If no roles are defined, display an information and return
		if(rdefset.getRoles().isEmpty()){
			showRoleDefinitionError();
			monitor.done();
			return;
		}
		
		monitor.subTask("Reading plan");
		
		// Get the plan
		Plan plan = (Plan)localResourceSet.getResource(URI.createPlatformResourceURI(file.getFullPath().toString(),
				true), true).getContents().get(0);
		
		monitor.worked(1);
		
		shouldGenerate = true;
		
		if(!plan.isMasterPlan())
			askForMissingMasterPlan();
		
		if(shouldGenerate){
			try{
				String container = PlanDesignerActivator.getDefault().getPreferenceStore().getString(PlanDesignerConstants.PREF_ROLE_DEFINITION_CONTAINER);
				
				String filename = PlanEditorUtils.removeFileExtension(file.getName()) + "RoleSet.rset";
				
				// Construct the path
				IPath path = ResourcesPlugin.getWorkspace().getRoot().findMember(container).getFullPath().append(filename);
				
				monitor.subTask((path.toFile().exists() ? "Loading " : "Creating ") +"roleset file");
				
				// Create a resource
				Resource roleSetResource = localResourceSet.createResource(URI.createPlatformResourceURI(path.toFile().getPath(),true));
				
				monitor.worked(1);
				
				monitor.subTask("Resolving dependencies");
				
				// Trigger to load all dependent resources
				loadDescendingPlans(plan);
				
				monitor.worked(1);
				
				monitor.subTask("Generation roleset");
				
				// Create the roleset
				RoleSet roleSet = AlicaFactory.eINSTANCE.createRoleSet();
				roleSet.setUsableWithPlanID(plan.getId());
				roleSet.setName("RoleSetFor" +plan.getName().replaceAll(" ", ""));
				roleSet.setDefault(true);
				
				// Assign all tasks for each role
				for(Role r : rdefset.getRoles())
					roleSet.getMappings().add(assignTasks(localResourceSet, r));
				
				// Add the plan to the resource
				roleSetResource.getContents().add(roleSet);
				
				monitor.worked(1);
				
				monitor.subTask("Saving");
				
				roleSetResource.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
				
				monitor.done();
			
			} catch(IOException e){
				status.add(new OperationStatus(
						OperationStatus.ERROR,
						PlanDesignerActivator.PLUGIN_ID, 94,
						"Read/Write failure", e));
			} finally{
				// Clean up
				localResourceSet.getResources().clear();
				localResourceSet = null;
				
				monitor.done();
			}
		}else{
			monitor.setCanceled(true);
			monitor.done();
		}
	}
	
	private void showRoleDefinitionError() {
		shell.getDisplay().syncExec(new Runnable(){
			public void run() {
				MessageDialog.openInformation(
						targetPart.getSite().getShell(), 
						"No Roles found", 
						"You didn't specify any roles for generating a roleset for. \n" +
						"Please define one or more roles first.");
				
			}
		});	}
//
//	/**
//	 * Creates a role with one characteristic attached
//	 * @param name
//	 * @param charKey
//	 * @param charVal
//	 * @return
//	 */
//	private Role createRole(String name, String charKey, CapValue charVal){
//		Role r = AlicaFactory.eINSTANCE.createRole();
//		r.setName(name);
//		if(charKey != null && charVal != null)
//			r.getCharacteristics().add(createCharacteristic(charKey, charVal));
//		
//		return r;
//	}
	
	private void loadDescendingPlans(Plan fromPlan){
		for(State s : fromPlan.getStates()){
			for(AbstractPlan plan : s.getPlans())
				if(plan instanceof Plan){
					plan.getName();
					loadDescendingPlans((Plan)plan);
				}
		}
	}
	
	public static Characteristic createCharacteristic(String key, CapValue charVal){
		Characteristic chara = AlicaFactory.eINSTANCE.createCharacteristic();
		chara.setName(key);
		chara.setValue(charVal);		
		return chara;
	}
	
	/**
	 * Creates a new RoleTaskassginement and associates it with the given role.
	 * After that it creates taskpriorities for each task.
	 * @param rSet
	 * @param r
	 * @return
	 */
	public static RoleTaskMapping assignTasks(ResourceSet rSet, Role r){
		List<Task> tasks = new ArrayList<Task>();
		
		for(Resource resource : rSet.getResources()){
			TreeIterator<EObject> iter = EcoreUtil.getAllContents(resource, true);
			while(iter.hasNext()){
				EObject next = iter.next();
				if(next instanceof Task)
					tasks.add((Task)next);
			}
		}
		
		RoleTaskMapping rtass = AlicaFactory.eINSTANCE.createRoleTaskMapping();
		rtass.setRole(r);
		
		for(Task t : tasks)
			rtass.getTaskPriorities().put(t.getId(), 0.5);
		
		return rtass;
	}
	
	private void askForMissingMasterPlan(){
		shell.getDisplay().syncExec(new Runnable(){
			public void run() {
				shouldGenerate =  MessageDialog.openQuestion(
						targetPart.getSite().getShell(), 
						"No masterplan", 
						"The selected plan is not a masterplan. You should either declare the plan\n" +
						"as master or walk up the hierarchy to get the masterplan.\n\n" +
						"Do you really want to generate a roleset for a plan which is not a masterplan?");
				
			}
		});
	}

	public void selectionChanged(IAction action, ISelection selection) {
		this.selection = selection;
	}

}
