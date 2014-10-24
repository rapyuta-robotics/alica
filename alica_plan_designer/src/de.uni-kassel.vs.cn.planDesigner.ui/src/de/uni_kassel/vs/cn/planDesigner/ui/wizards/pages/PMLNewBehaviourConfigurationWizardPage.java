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
package de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages;

import org.eclipse.emf.transaction.RunnableWithResult;
import org.eclipse.jface.dialogs.IDialogPage;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.ModifyEvent;
import org.eclipse.swt.events.ModifyListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;

import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

/**
 * This page allows the user to create a new BehaviourConfiguration for a given
 * Behaviour which can be from previous pages or from extern.
 */
public class PMLNewBehaviourConfigurationWizardPage extends WizardPage {
	
	private Text configurationNameText;

	final private Behaviour behaviour;
	PMLNewBehaviourWizardPage behPage;
	
	private PMLTransactionalEditingDomain editingDomain;

	/**
	 * Constructor for SampleNewWizardPage.
	 * 
	 * @param pageName
	 */
	public PMLNewBehaviourConfigurationWizardPage(PMLTransactionalEditingDomain editingDomain, Behaviour behaviour) {
		super("New Behaviourconfiguration");
		setTitle("New Behaviourconfiguration");
		setDescription("Create a new Behaviourconfiguration");
		this.behaviour = behaviour;
		this.editingDomain = editingDomain;
	}
	public PMLNewBehaviourConfigurationWizardPage(PMLTransactionalEditingDomain editingDomain, Behaviour behaviour, PMLNewBehaviourWizardPage behPage) {
		super("New Behaviourconfiguration");
		setTitle("New Behaviourconfiguration");
		setDescription("Create a new Behaviourconfiguration");
		this.behaviour = behaviour;
		this.behPage = behPage;
		this.editingDomain = editingDomain;
	}
	

	/**
	 * @see IDialogPage#createControl(Composite)
	 */
	public void createControl(Composite parent) {
		Composite container = new Composite(parent, SWT.NULL);
		GridLayout layout = new GridLayout();
		container.setLayout(layout);
		layout.numColumns = 3;
		layout.verticalSpacing = 9;
		
		Label label = new Label(container, SWT.NULL);
		label = new Label(container, SWT.WRAP);
		label.setText("Enter a meaningful name for the new Behaviourconfiguration e.g. \"DriveToLeftCorner\"");
		
		GridData gd = new GridData(GridData.FILL_HORIZONTAL);
		gd = new GridData(GridData.FILL_HORIZONTAL);
		gd.horizontalSpan = 3;
		label.setLayoutData(gd);
		
		label = new Label(container, SWT.WRAP);
		label.setText("Configuration name:");
		label.setLayoutData(new GridData());

		configurationNameText = new Text(container, SWT.BORDER | SWT.SINGLE);
		configurationNameText.setLayoutData(new GridData(SWT.FILL,SWT.CENTER,true,false,2,1));
		configurationNameText.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				dialogChanged();
			}
		});

		initialize();
		dialogChanged();
		setControl(container);
	}
	
	private void initialize(){
		if(this.behPage != null)
		{
			configurationNameText.setText(this.behPage.getName() + "Default");
		} else
		{
			configurationNameText.setText(this.behaviour.getName() + "Default");
		}
	}


	/**
	 * Ensures that both text fields are set.
	 */
	private void dialogChanged() {
		String configName = getConfigurationName();

		if (configName.length() == 0) {
			updateStatus("File name must be specified");
			return;
		}
		
		if(behaviourContainsConfigurationName(configName)){
			updateStatus("There is already a configuration with the name \"" +configName +"\" in " +
					"the Behaviour. Maybe you don't need to add a new BehaviourConfiguration.");
			return;
		}
			
			
		updateStatus(null);
	}
	
	private boolean behaviourContainsConfigurationName(final String name){
		boolean result = false;
		try {
			result = (Boolean)editingDomain.runExclusive(new RunnableWithResult.Impl<Boolean>(){
				public void run() {
					setResult(Boolean.FALSE);
					for(BehaviourConfiguration conf : behaviour.getConfigurations())
						if(conf.getName().equalsIgnoreCase(name)){
							setResult(Boolean.TRUE);
							break;
						}
				}
			});
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		return result;
	}

	private void updateStatus(String message) {
		setErrorMessage(message);
		setPageComplete(message == null);
	}

	public String getConfigurationName() {
		return configurationNameText.getText();
	}
}