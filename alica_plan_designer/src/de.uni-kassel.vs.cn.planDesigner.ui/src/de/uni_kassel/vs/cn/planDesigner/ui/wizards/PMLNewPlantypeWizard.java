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
import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.operation.IRunnableWithProgress;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.wizard.Wizard;
import org.eclipse.ui.INewWizard;
import org.eclipse.ui.IWorkbench;
import org.eclipse.ui.IWorkbenchWizard;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLComposePlanTypeWizardPage;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewPlanTypeWizardPage;

/**
 * Wizard to generate a new .pty file. 
 */

public class PMLNewPlantypeWizard extends Wizard implements INewWizard {
	
	private PMLComposePlanTypeWizardPage composePlantypePage;
	private PMLNewPlanTypeWizardPage newPlantypePage;
	
	private ISelection selection;
	
	private ResourceSet rSet;
	
	private PMLTransactionalEditingDomain domain;
	
	private PlanType newPlanType;
	
	public PMLNewPlantypeWizard()
	{
		this(AlicaFactory.eINSTANCE.createPlanType());
	}
	/**
	 * Constructor for PlanmodellerNewPlanWizard.
	 */
	public PMLNewPlantypeWizard(PlanType newPlanType) {
		super();
		this.newPlanType = newPlanType;
		setNeedsProgressMonitor(true);
		setWindowTitle("New Plantype");
		setDefaultPageImageDescriptor(PlanDesignerActivator.getDefault().getImageRegistry().getDescriptor(PlanDesignerConstants.ICON_PLANTYPE_24));
	}
	
	/**
	 * Adding the composePlantypePage to the wizard.
	 */

	public void addPages() {
		newPlantypePage = new PMLNewPlanTypeWizardPage(selection);
		composePlantypePage = new PMLComposePlanTypeWizardPage(getEditingDomain(),getResourceSet());
		addPage(newPlantypePage);
		addPage(composePlantypePage);
	}
	
	/**
	 * This method is called when 'Finish' button is pressed in
	 * the wizard. We will create an operation and run it
	 * using wizard as execution context.
	 */
	public boolean performFinish() {
		final String containerName = newPlantypePage.getContainerName();
		final String fileName = PlanEditorUtils.removeFileExtension(newPlantypePage.getFileName()) + ".pty";
		
		IRunnableWithProgress op = new IRunnableWithProgress() {
			public void run(IProgressMonitor monitor) throws InvocationTargetException {
				try {
					doFinish(containerName, fileName, monitor);
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
	 * The worker method. 
	 */
	private void doFinish(String containerName, final String fileName, IProgressMonitor monitor)
																	throws CoreException {
		// create a sample file
		monitor.beginTask("Creating " + fileName, 3);
		
		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		IResource resource = root.findMember(new Path(containerName));
		if (!resource.exists() || !(resource instanceof IContainer)) {
			throwCoreException("Container \"" + containerName + "\" does not exist.");
		}
		IContainer container = (IContainer) resource;
		final IFile file = container.getFile(new Path(fileName));
		
		monitor.worked(1);
		monitor.subTask("Creating plantype contents");
		
		domain.getCommandStack().execute(new AbstractCommand(){

			@Override
			public boolean canUndo() {
				return false;
			}

			public void execute() {
				// Create a new resource for the plantype
				Resource planTypeResource = getResourceSet().createResource(URI.createPlatformResourceURI(file.getFullPath().toOSString(), true));
				initPlantypeContents(PlanEditorUtils.removeFileExtension(fileName));
				planTypeResource.getContents().add(newPlanType);
				
				try {
					planTypeResource.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
				} catch (IOException e) {
					e.printStackTrace();
				}
			}

			public void redo() {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			protected boolean prepare() {
				return true;
			}
			
		});
		
		monitor.done();
	}
	
	private void initPlantypeContents(String name) {
		newPlanType.setName(name);
		
		// Get the set of plans from the compose page
		Set<AnnotatedPlan> plansToAdd = composePlantypePage.getPlantypeViewerList();
		for(AnnotatedPlan p : plansToAdd)
		{
			newPlanType.getPlans().add(p);
		}
	}

	private void throwCoreException(String message) throws CoreException {
		IStatus status =
			new Status(IStatus.ERROR, "de.uni_kassel.vs.cn.planDesigner.ui", IStatus.OK, message, null);
		throw new CoreException(status);
	}
	
	public PlanType getCreatedPlanType()
	{
		return newPlanType;
	}

	/**
	 * We will accept the selection in the workbench to see if
	 * we can initialize from it.
	 * @see IWorkbenchWizard#init(IWorkbench, IStructuredSelection)
	 */
	public void init(IWorkbench workbench, IStructuredSelection selection) {
		this.selection = selection;
	}

	private ResourceSet getResourceSet() {
		if(rSet == null)
			rSet = getEditingDomain().getResourceSet();
		
		return rSet;
	}

	private PMLTransactionalEditingDomain getEditingDomain() {
		if(domain == null)
			domain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		
		return domain;
	}

}