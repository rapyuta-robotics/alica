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

import java.lang.reflect.InvocationTargetException;

import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.operation.IRunnableWithProgress;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.wizard.Wizard;
import org.eclipse.ui.INewWizard;
import org.eclipse.ui.IWorkbench;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewBehaviourConfigurationWizardPage;

public class PMLNewBehaviourConfigurationWizard extends Wizard implements INewWizard {

	private PMLNewBehaviourConfigurationWizardPage behaviourConfigurationPage;
	
	private PMLTransactionalEditingDomain editingDomain;
	
	private Behaviour behaviour;
	
	
	private BehaviourConfiguration createdConfiguration;
	
	public PMLNewBehaviourConfigurationWizard(Behaviour behaviour) {
		super();
		setNeedsProgressMonitor(true);
		setWindowTitle("New Behaviourconfiguration");
		
		this.behaviour = behaviour;
		// Connect to the editingDomain
		editingDomain =  (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
				PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
	}

	@Override
	public boolean performFinish() {
		System.out.println("WIZARDCONFO");
		final String configurationName = behaviourConfigurationPage.getConfigurationName();
		IRunnableWithProgress op = new IRunnableWithProgress() {
			public void run(IProgressMonitor monitor) throws InvocationTargetException {
					doFinish(configurationName, monitor);
					monitor.done();
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
	
	private void doFinish(String configurationName, IProgressMonitor monitor) {
		monitor.beginTask("Creating new Behaviourconfiguration",1);
		
		createdConfiguration = AlicaFactory.eINSTANCE.createBehaviourConfiguration();
		createdConfiguration.setName(configurationName);
		
		monitor.worked(1);
	}
	
	/**
	 * Adding the page to the wizard.
	 */

	public void addPages() {
		behaviourConfigurationPage = new PMLNewBehaviourConfigurationWizardPage(editingDomain, behaviour );
		addPage(behaviourConfigurationPage);
	}

	

	public BehaviourConfiguration getCreatedConfiguration() {
		return createdConfiguration;
	}

	@Override
	public void init(IWorkbench workbench, IStructuredSelection selection) {
		// TODO Auto-generated method stub
		
	}
	
	

}
