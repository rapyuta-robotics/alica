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
import java.util.Collections;

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
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.transaction.RecordingCommand;
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
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewBehaviourConfigurationWizardPage;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewBehaviourWizardPage;

public class PMLNewBehaviourWizard extends Wizard implements INewWizard {

	private ISelection selection;
	
	private PMLNewBehaviourWizardPage behaviourPage;
	private PMLNewBehaviourConfigurationWizardPage configurationWizardPage;
	
	private PMLTransactionalEditingDomain editingDomain;
	
	private Behaviour behaviour;
	
	private BehaviourConfiguration createdConfiguration;
	
	public PMLNewBehaviourWizard(){
		this(AlicaFactory.eINSTANCE.createBehaviour());
	}

	public PMLNewBehaviourWizard(Behaviour behaviour) {
		super();
		setNeedsProgressMonitor(true);
		setWindowTitle("New Behaviour");
		
		// Connect to the editingDomain
		this.editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
				PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		this.behaviour = behaviour;
	}

	@Override
	public boolean performFinish() {
		final String containerName = behaviourPage.getContainerName();
		final String fileName = PlanEditorUtils.removeFileExtension(behaviourPage.getBehaviourName()) + ".beh";
		final String configName = configurationWizardPage.getConfigurationName();
	
		IRunnableWithProgress op = new IRunnableWithProgress() {
			public void run(IProgressMonitor monitor) throws InvocationTargetException {
				try {
					doFinish(containerName, fileName,configName, monitor);
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
	
	private void doFinish(String containerName, String fileName,String configName,IProgressMonitor monitor)
																	throws CoreException {
		// create the behaviour file
		monitor.beginTask("Creating " + fileName, 1);
		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		IResource resource = root.findMember(new Path(containerName));
		if (!resource.exists() || !(resource instanceof IContainer)) {
			throwCoreException("Container \"" + containerName + "\" does not exist.");
		}
	
		IContainer container = (IContainer) resource;
		final IFile file = container.getFile(new Path(fileName));
		if (file.exists()) {
			// This should not be the case since the user should have specified a filename
			// which is equivalent to the behaviourname
			// file.setContents(stream, true, true, monitor);
			System.err.println("Overwriting existing behaviour!!!");
		} else{
			// Create an empty file
			file.create(null, true, monitor);
			// Init the file with the given plan
			initFileWithBehaviour(file, configName);
		}
		monitor.worked(1);
	}
	
	/**
	 * Adding the page to the wizard.
	 */
	public void addPages() {
		configurationWizardPage = new PMLNewBehaviourConfigurationWizardPage(editingDomain, behaviour, behaviourPage);
		behaviourPage = new PMLNewBehaviourWizardPage(selection , this);
		addPage(behaviourPage);
		addPage(configurationWizardPage);
	}

	public PMLNewBehaviourConfigurationWizardPage getBehConfWizardPage() {
		return configurationWizardPage;
	}
	/**
	 * We will accept the selection in the workbench to see if
	 * we can initialise from it.
	 * @see IWorkbenchWizard#init(IWorkbench, IStructuredSelection)
	 */
	public void init(IWorkbench workbench, IStructuredSelection selection) {
		this.selection = selection;
	}

	private void throwCoreException(String message) throws CoreException {
		IStatus status =
			new Status(IStatus.ERROR, "de.uni_kassel.vs.cn.planDesigner.ui", IStatus.OK, message, null);
		throw new CoreException(status);
	}
	
	private void initFileWithBehaviour(final IFile file, final String configName){
		final Resource res = editingDomain.getResourceSet().createResource(URI.createPlatformResourceURI(file.getFullPath().toOSString(), true));
		
		editingDomain.getCommandStack().execute(new RecordingCommand(editingDomain){
			@Override
			protected void doExecute() {
				Behaviour b = behaviour != null ? behaviour : AlicaFactory.eINSTANCE.createBehaviour();
				b.setName(file.getName().substring(0,file.getName().lastIndexOf(".")));
				
				
				// Create the behaviourConfiguration
				createdConfiguration = AlicaFactory.eINSTANCE.createBehaviourConfiguration();
				createdConfiguration.setName(configName);
				
				// Add it to the Behaviour
				b.getConfigurations().add(createdConfiguration);
				res.getContents().add(b);
				
				try {
					res.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		});
	}

	public BehaviourConfiguration getCreatedConfiguration() {
		return createdConfiguration;
	}
}
