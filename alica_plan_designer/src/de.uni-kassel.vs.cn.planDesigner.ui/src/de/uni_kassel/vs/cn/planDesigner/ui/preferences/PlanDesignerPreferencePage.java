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
package de.uni_kassel.vs.cn.planDesigner.ui.preferences;

import java.util.HashMap;
import java.util.Map;

import org.eclipse.core.runtime.IPath;
import org.eclipse.jface.preference.IPreferenceStore;
import org.eclipse.jface.preference.PreferencePage;
import org.eclipse.jface.window.Window;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;
import org.eclipse.ui.IWorkbench;
import org.eclipse.ui.IWorkbenchPreferencePage;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.ContainerDialog;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class PlanDesignerPreferencePage extends PreferencePage implements
		IWorkbenchPreferencePage {
	
	private static final int ROLE_DEF_TEXT = 0;
	private static final int CAP_DEF_TEXT = 3;
	private static final int CODEGEN_BASE_TEXT = 1;
	private static final int MISCELLANEOUS_TEXT = 2;
	
	private Map<Integer, Text> textFieldmap = new HashMap<Integer, Text>();
	
	@Override
	protected Control createContents(Composite parent) {
		final Composite container = new Composite(parent, SWT.NONE);
		container.setLayout(new GridLayout(1,true));
		
		// Add the preference for the roledefinition path
		createPathPreference(container, 
				"Role definition settings", 
				"Directory which will hold the role definition file. This is also the directory where\n" +
				"Rolesets will be saved in", 
				ROLE_DEF_TEXT);
		// Add the preference for the capabilitydefinition path
		createPathPreference(container, 
				"Capability definition settings", 
				"Directory which will hold the capability definition file. This is usually the directory where\n" +
				"Rolesets will be saved in", 
				CAP_DEF_TEXT);
		
		// Add the preference for the codegen path
		createPathPreference(container, 
				"Codegeneration settings", 
				"Base directory for codegeneration", 
				CODEGEN_BASE_TEXT);
		
		
		// Add the preference for misc path
		createPathPreference(container,
				"Taskrepository settings",
				"Directory which will hold the taskrepository file.",
				MISCELLANEOUS_TEXT);
		
		initializeValues();
		
		return container;
	}

	private void createPathPreference(final Composite container, String groupLabel, String description, int associatedTextField) {
		Group group;
		Label label;
		Label pathLabel;
		Button browseButton;
		GridData gData;
		
		group = new Group(container, SWT.NONE);
		group.setLayout(new GridLayout(3,false));
		group.setLayoutData(new GridData(SWT.FILL,SWT.CENTER,true,false));
		group.setText(groupLabel);
		
		label = new Label(group, SWT.WRAP);
		label.setText(description);
		
		pathLabel = new Label(group, SWT.NONE);
		pathLabel.setText("Path:");
		
		final Text textField = new Text(group, SWT.BORDER | SWT.READ_ONLY);
		textFieldmap.put(associatedTextField, textField);
		
		browseButton = new Button(group, SWT.PUSH);
		browseButton.setText("Browse");
		browseButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				ContainerDialog dia = new ContainerDialog(PlanDesignerPreferencePage.this.getShell());
				
				if(dia.open() == Window.OK){
					IPath path = dia.getSelectedPath();
					textField.setText(path.toString());
				}
			}
		});
		
		// Layout
		gData = new GridData(SWT.FILL,SWT.FILL,true,true);
		gData.horizontalSpan = 3;
		label.setLayoutData(gData);
		
		pathLabel.setLayoutData(new GridData(SWT.FILL,SWT.CENTER,false,false));
		textField.setLayoutData(new GridData(SWT.FILL,SWT.CENTER,true,false));
		browseButton.setLayoutData(new GridData(SWT.FILL,SWT.CENTER,false,false));
	}

	private void initializeValues() {
		IPreferenceStore store = getPreferenceStore();
		textFieldmap.get(ROLE_DEF_TEXT).setText(store.getString(PlanDesignerConstants.PREF_ROLE_DEFINITION_CONTAINER));
		textFieldmap.get(CAP_DEF_TEXT).setText(store.getString(PlanDesignerConstants.PREF_CAPABILITY_DEFINITION_CONTAINER));
		textFieldmap.get(CODEGEN_BASE_TEXT).setText(store.getString(PlanDesignerConstants.PREF_CODEGEN_BASE_PATH));
		textFieldmap.get(MISCELLANEOUS_TEXT).setText(store.getString(PlanDesignerConstants.PREF_MISC_PROJECT_WORKSPACE_PATH));
	}
	
	private void initializeDefaults(){
		IPreferenceStore store = getPreferenceStore();
		textFieldmap.get(ROLE_DEF_TEXT).setText(store.getDefaultString(PlanDesignerConstants.PREF_ROLE_DEFINITION_CONTAINER));
		textFieldmap.get(CAP_DEF_TEXT).setText(store.getDefaultString(PlanDesignerConstants.PREF_CAPABILITY_DEFINITION_CONTAINER));
		textFieldmap.get(CODEGEN_BASE_TEXT).setText(store.getDefaultString(PlanDesignerConstants.PREF_CODEGEN_BASE_PATH));
		textFieldmap.get(MISCELLANEOUS_TEXT).setText(store.getDefaultString(PlanDesignerConstants.PREF_MISC_PROJECT_WORKSPACE_PATH));
	}
	
	@Override
	protected void performDefaults() {
		super.performDefaults();
		initializeDefaults();
	}
	
	
	@Override
	public boolean performOk() {
		storeValues();
		return super.performOk();
	}
	
	private void storeValues(){
		IPreferenceStore store = getPreferenceStore();
		
		store.setValue(PlanDesignerConstants.PREF_ROLE_DEFINITION_CONTAINER, textFieldmap.get(ROLE_DEF_TEXT).getText());
		store.setValue(PlanDesignerConstants.PREF_CAPABILITY_DEFINITION_CONTAINER, textFieldmap.get(CAP_DEF_TEXT).getText());
		store.setValue(PlanDesignerConstants.PREF_CODEGEN_BASE_PATH, textFieldmap.get(CODEGEN_BASE_TEXT).getText());
		store.setValue(PlanDesignerConstants.PREF_MISC_PROJECT_WORKSPACE_PATH, textFieldmap.get(MISCELLANEOUS_TEXT).getText());
	}
	
	@Override
	protected IPreferenceStore doGetPreferenceStore() {
		return PlanDesignerActivator.getDefault().getPreferenceStore();
	}
	
	

	public void init(IWorkbench workbench) {
	}
}
