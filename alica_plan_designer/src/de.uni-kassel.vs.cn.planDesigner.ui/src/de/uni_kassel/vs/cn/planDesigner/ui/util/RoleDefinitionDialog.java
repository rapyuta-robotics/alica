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
package de.uni_kassel.vs.cn.planDesigner.ui.util;

import java.awt.Event;
import java.util.Collections;
import java.util.List;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.RemoveCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.viewers.ILabelProvider;
import org.eclipse.jface.viewers.ILabelProviderListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.window.IShellProvider;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.DisposeEvent;
import org.eclipse.swt.events.DisposeListener;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.layout.FormLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Combo;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Widget;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.CapValue;
import de.uni_kassel.vs.cn.planDesigner.alica.Capability;
import de.uni_kassel.vs.cn.planDesigner.alica.CapabilityDefinitionSet;
import de.uni_kassel.vs.cn.planDesigner.alica.Characteristic;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.alica.Role;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleDefinitionSet;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.EditController;

public class RoleDefinitionDialog extends Dialog {
	
	private class RoleLabelProvider implements ILabelProvider{

		public Image getImage(Object element) {
			if(element instanceof Role)
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_ROLE_16);
			else if(element instanceof Characteristic)
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_CHARACTERISTIC_16);
			else
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_UNKNOWN_TYPE);
		}

		public String getText(Object element) {
			if(element instanceof PlanElement){
				return ((PlanElement)element).getName();
			}else
				return "Unknown element";
		}

		public void addListener(ILabelProviderListener listener) {
		}

		public void dispose() {
		}

		public boolean isLabelProperty(Object element, String property) {
			// TODO Optimize this
			return true;
		}

		public void removeListener(ILabelProviderListener listener) {
		}
		
	}
	
	private class RoleContentProvider implements ITreeContentProvider{

		public Object[] getChildren(Object parentElement) {
			if(parentElement instanceof Role){
				return ((Role)parentElement).getCharacteristics().toArray();
			}else
				return null;
		}

		public Object getParent(Object element) {
			if(element instanceof EObject)
				return ((EObject)element).eContainer();
			else
				return null;
		}

		public boolean hasChildren(Object element) {
			if(element instanceof Role)
				return !((Role)element).getCharacteristics().isEmpty();
			else
				return false;
		}

		public Object[] getElements(Object inputElement) {
			// The input is expected to be an RoleDefinitionSet
			if(inputElement instanceof RoleDefinitionSet){
				return ((RoleDefinitionSet)inputElement).getRoles().toArray();
			}
			return new Object[]{"Input invalid"};
		}

		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
			
		}
		
	}
	
	private Group nothingToDisplayGroup;
	
	private Group roleDataGroup;
	
	private Group characterDataGroup;
	
	private TreeViewer roleViewer;
	
	private Button addRoleButton;
	private Button removeButton;
	private Button addCharButton;
	
	private Text roleNameText;
		
	private Combo keyCombo;
	private Combo valueCombo;
	
	private Text weightText;
	
	private Composite container;
	
	private RoleDefinitionSet input;
	private CapabilityDefinitionSet cdSet;
	
	private Object currentSelection;
	
	private EditController editController;

	private PMLTransactionalEditingDomain domain;
	private Shell shell;
	
	public RoleDefinitionDialog(IShellProvider provider, PMLTransactionalEditingDomain domain) {
		super(provider);
		this.domain = domain;
	}
	
	@Override
	protected Control createDialogArea(Composite parent) {
		container = (Composite)super.createDialogArea(parent);

		// Layout for the composite
		GridLayout gLayout = new GridLayout(2,false);
		container.setLayout(gLayout);
		
		// Initialize the header
		initializeHeader(container);
		
		// Init the left side. This will always be the same
		Composite left = createLeftSide(container);
		GridData gData = new GridData(SWT.FILL,SWT.FILL,true,true);
		left.setLayoutData(gData);
		
		// Initially we will display the nothing to display composite
		displayComposite(getNothingToDisplayGroup());
		
		container.addDisposeListener(new DisposeListener(){
			public void widgetDisposed(DisposeEvent e) {
				getEditController().removeFromObjects();
			}
		});
		
		return container;
	}
	
	/**
	 * Sets the input of this dialog. The given Resources content is expected to be an
	 * instance of P
	 * @param res
	 */
	public void setInput(Resource resRole, Resource resCap){
		this.input = (RoleDefinitionSet)resRole.getContents().get(0);
		this.cdSet = (CapabilityDefinitionSet)resCap.getContents().get(0);
		registerController(input);
		
		// Apply the input to the roleViewer
		getRoleViewer().setInput(input);
		
		if(!input.getRoles().isEmpty()){
			// Select the first entry
			getRoleViewer().setSelection(new StructuredSelection(input.getRoles().get(0)));
		}
		
	}
	
	private void registerController(RoleDefinitionSet roleDef){
		EditController controller = getEditController();
		
		controller.addToObject(roleDef);
		
		List<Role> roles =  roleDef.getRoles();
		for(Role r : roles){
			controller.addToObject(r);
			for(Characteristic c : r.getCharacteristics())
				controller.addToObject(c);
		}		
		
		populateCombos();
	}
	
	private void populateCombos()
	{		
		keyCombo.removeAll();		
		List<Capability> capabilities =  cdSet.getCapabilities();
		for(Capability c : capabilities)
		{
			//System.out.println(c.getName());
			keyCombo.add(c.getName());
			keyCombo.setData(c.getName(), c);
			//keyCombo.setItem(i++, c.getName());
		}
		//keyCombo.addTraverseListener(listener)
		keyCombo.addSelectionListener(new SelectionAdapter() 
		{
		      public void widgetSelected(SelectionEvent e) 
		      {
		    	  populateCapvalue();
		      }
		});
		
	}
	
	private void populateCapvalue()
	{
		Capability oldCap = (Capability)keyCombo.getData(keyCombo.getText());		
		valueCombo.removeAll();
		List<Capability> capabilities =  cdSet.getCapabilities();

		if(capabilities.contains(oldCap))
		{
			List<CapValue> capValues = oldCap.getCapValues();
			for(CapValue cv : capValues)
			{
				valueCombo.add(cv.getName());
				valueCombo.setData(cv.getName(), cv);			
			}		
		}
	}
	
	/** 
	 * Displays the given composite and hides the others
	 * @param compToDisplay
	 */
	private void displayComposite(Composite compToDisplay){
		GridData gData = null;

		gData = (GridData)getRoleDataGroup().getLayoutData();
		gData.exclude = compToDisplay.equals(getRoleDataGroup()) ? false : true;
		
		gData = (GridData)getCharacterDataGroup().getLayoutData();
		gData.exclude = compToDisplay.equals(getCharacterDataGroup()) ? false : true;
		
		gData = (GridData)getNothingToDisplayGroup().getLayoutData();
		gData.exclude = compToDisplay.equals(getNothingToDisplayGroup()) ? false : true;
		
		compToDisplay.getParent().layout();
		
		// TODO: That could be done better!
		if(compToDisplay.equals(getCharacterDataGroup())){
			getNothingToDisplayGroup().setVisible(false);
			getRoleDataGroup().setVisible(false);
		}else if(compToDisplay.equals(getNothingToDisplayGroup())){
			getCharacterDataGroup().setVisible(false);
			getRoleDataGroup().setVisible(false);
		}else{
			getNothingToDisplayGroup().setVisible(false);
			getCharacterDataGroup().setVisible(false);
		}
		compToDisplay.setVisible(true);
	}
	
	/**
	 * Refrehes the UI with information freshly obtained from the model
	 */
	private void refreshRoleGroup(){
		Role r = (Role)currentSelection;
		roleNameText.setText(r.getName());
	}
	
	/**
	 * Refrehes the UI with information freshly obtained from the model
	 */
	private void refreshCharacteristicGroup(){		
		Characteristic chara = (Characteristic)currentSelection;
		//charNameText.setText(chara.getName());
		
		if(chara.getCapability() != null) 
		{
			keyCombo.setText(chara.getCapability().getName());
			populateCapvalue();
		}
		else
		{
			keyCombo.setText("");
			valueCombo.removeAll();
		}	
		if(chara.getValue() != null) 
			valueCombo.setText(chara.getValue().getName());
//		else
//			valueCombo.setText("");
		weightText.setText(((Double)chara.getWeight()).toString());
	}
	
	private void initializeHeader(Composite parent){
		Composite descContainer = new Composite(parent, SWT.NONE);
		GridData gData = new GridData(SWT.FILL,SWT.FILL,true,false);
		gData.horizontalSpan = 2;
		descContainer.setLayoutData(gData);
		descContainer.setLayout(new FormLayout());
		
		Label imgLabel = new Label(descContainer,SWT.NONE);
		imgLabel.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_ROLE_DEFINITION_64));
		imgLabel.setLayoutData(new FormData());
		
		Label desc = new Label(descContainer, SWT.NONE);
		desc.setText("Define Roles and their Capabilities. \n\nThese roles can be references while associating roles with tasks. " +
				"\nNote: If you add/remove roles, ensure to adjust your rolesets!");
		FormData fData = new FormData();
		fData.left = new FormAttachment(imgLabel, 20);
		desc.setLayoutData(fData);
	}
	
	private void intializeButtonsContainer(Composite parent){
		Composite buttonsContainer = new Composite(parent,SWT.NONE);
		buttonsContainer.setLayoutData(new GridData(SWT.CENTER,SWT.CENTER,false,false));
		FormLayout fLayout = new FormLayout();
		fLayout.spacing = 15;
		buttonsContainer.setLayout(fLayout);
		
		addRoleButton = new Button(buttonsContainer, SWT.PUSH);
		addRoleButton.setText("Add Role");
		addRoleButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				Role r = AlicaFactory.eINSTANCE.createRole();
				// Register the editController as adapter
				getEditController().addToObject(r);
				
				domain.getCommandStack().execute(
						CreateChildCommand.create(
								domain, 
								input, 
								new CommandParameter(input,AlicaPackage.eINSTANCE.getRoleDefinitionSet_Roles(),r), 
								Collections.emptyList()));
				
				// Select the new role
				getRoleViewer().setSelection(new StructuredSelection(r),true);
				removeButton.setEnabled(true);
			}
		});
		
		removeButton = new Button(buttonsContainer, SWT.PUSH);
		removeButton.setText("Remove");
		removeButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				// Remove the editController from the object
				getEditController().removeFromObject((EObject)currentSelection);
				
				domain.getCommandStack().execute(
						RemoveCommand.create(
								domain, 
								Collections.singleton(currentSelection)));
				
				if(input.getRoles().isEmpty()){
					removeButton.setEnabled(false);
					getRoleViewer().setSelection(new StructuredSelection(StructuredSelection.EMPTY));
				} else{
					removeButton.setEnabled(true);
					
					getRoleViewer().setSelection(new StructuredSelection(input.getRoles().get(0)));
				}
					
			}
		});
		
		addCharButton = new Button(buttonsContainer, SWT.PUSH);
		addCharButton.setText("Add Capabilities");
		addCharButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				Role r = (Role)currentSelection;
				Characteristic c = AlicaFactory.eINSTANCE.createCharacteristic();
			
				// Register the editController as adapter
				getEditController().addToObject(c);
				
				domain.getCommandStack().execute(
						CreateChildCommand.create(
								domain, 
								r, 
								new CommandParameter(r,AlicaPackage.eINSTANCE.getRole_Characteristics(),c), 
								Collections.emptyList()));
				
				// Select the new characteristic
				getRoleViewer().setSelection(new StructuredSelection(c),true);
			}
		});
		addCharButton.setEnabled(false);
		
		// Layout for the buttonsContainer
		FormData fData = new FormData();
		fData.top = new FormAttachment();
		fData.width = 140;
		addRoleButton.setLayoutData(fData);
		
		fData = new FormData();
		fData.top = new FormAttachment(addRoleButton);
		fData.width = 140;
		addCharButton.setLayoutData(fData);
		
		fData = new FormData();
		fData.top = new FormAttachment(addCharButton,15);
		fData.width = 140;
		removeButton.setLayoutData(fData);
		
	}

	@Override
	protected void configureShell(Shell newShell) {
		super.configureShell(newShell);
		newShell.setText("Role Definition");
		newShell.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_ROLE_DEFINITION_16));
		this.shell = newShell;
	}

	private Group getNothingToDisplayGroup() {
		if(nothingToDisplayGroup == null){
			nothingToDisplayGroup = createNothingToDisplayGroup(container);
		}
		return nothingToDisplayGroup;
	}

	private Group getRoleDataGroup() {
		if(roleDataGroup == null){
			roleDataGroup = createRoleDataGroup(container);
		}
		return roleDataGroup;
	}

	private Group getCharacterDataGroup() {
		if(characterDataGroup == null){
			characterDataGroup = createCharacterDataGroup(container);
		}
		return characterDataGroup;
	}

	private Group createRoleDataGroup(Composite parent) {
		Group rDataComp = new Group(parent, SWT.NONE);
		rDataComp.setText("Role");
		rDataComp.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
		rDataComp.setLayout(PlanEditorUtils.createFormLayout());
		
		Label nameLabel = new Label(rDataComp, SWT.NONE);
		nameLabel.setText("Name:");
		
		roleNameText = new Text(rDataComp, SWT.BORDER);
		roleNameText.addListener(SWT.FocusOut, getEditController());
		
		
		// Layout
		FormData fData = new FormData();
		fData.width = 40;
		nameLabel.setLayoutData(fData);
		
		fData = new FormData();
		fData.left = new FormAttachment(nameLabel);
		fData.width = 200;
		roleNameText.setLayoutData(fData);
		
		return rDataComp;
	}
	
	private Group createNothingToDisplayGroup(Composite parent) {
		Group ntdComposite = new Group(parent, SWT.NONE);
		ntdComposite.setText("No Information available");
		FillLayout fLayout = new FillLayout();
		fLayout.marginHeight = 15;
		fLayout.marginHeight = 15;
		ntdComposite.setLayout(fLayout);
		ntdComposite.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
		
		Label l = new Label(ntdComposite, SWT.WRAP);
		l.setText("Nothing to display. Create a role by clicking \"Add Role\"");
		
		return ntdComposite;
	}

	private Group createCharacterDataGroup(Composite parent) {
		Group cDataComp = new Group(parent,SWT.NONE);
		cDataComp.setText("Capabilities");
		cDataComp.setLayout(PlanEditorUtils.createFormLayout());
		cDataComp.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
		
//		Label nameLabel = new Label(cDataComp, SWT.NONE);
//		nameLabel.setText("Name:");
		
//		charNameText = new Text(cDataComp, SWT.BORDER);
//		charNameText.addListener(SWT.FocusOut, getEditController());
		
		Label keyLabel = new Label(cDataComp, SWT.NONE);
		keyLabel.setText("Key:");
		
//		keyText = new Text(cDataComp, SWT.BORDER);
//		keyText.addListener(SWT.FocusOut, getEditController());
		
//		Label keyLabel1 = new Label(cDataComp, SWT.NONE);
//		keyLabel1.setText("Key1:");
		
		keyCombo = new Combo(cDataComp, SWT.BORDER | SWT.DROP_DOWN | SWT.READ_ONLY );
		keyCombo.addListener(SWT.FocusOut, getEditController());
//		keyCombo.addSelectionListener(new SelectionAdapter() {
//		      public void widgetSelected(SelectionEvent e) {
//		    	  int index =  keyCombo.getSelectionIndex();
//		    	  System.out.println("INT############### " + index);
//		    	  currentSelection = keyCombo.getItem(index);
//		    	  if(currentSelection instanceof Capability)
//		    	  {
//		    		  System.out.println("FUNKZT");
//		    	  }
//		      }
//		});
		Label valueLabel = new Label(cDataComp, SWT.NONE);
		valueLabel.setText("Value:");
		
		valueCombo = new Combo(cDataComp, SWT.BORDER | SWT.READ_ONLY);
		valueCombo.addListener(SWT.FocusOut, getEditController());
		Label weightLabel = new Label(cDataComp, SWT.NONE);
		weightLabel.setText("Weight:");
		
		weightText = new Text(cDataComp, SWT.BORDER);
		weightText.addListener(SWT.FocusOut, getEditController());

		
		// Layout of CharNameLabel
//		FormData fData = new FormData();
//		fData.width = 50;
//		nameLabel.setLayoutData(fData);
		
		// Layout of CharNameText
//		fData = new FormData();
//		fData.left = new FormAttachment(nameLabel);
//		fData.width = 200;
//		charNameText.setLayoutData(fData);
		
		// Layout of keyLabel
		FormData fData = new FormData();
		fData.width = 50;
		keyLabel.setLayoutData(fData);
		
		fData = new FormData();
		fData.left = new FormAttachment(keyLabel);
		fData.width = 200;
		keyCombo.setLayoutData(fData);
		
		// Layout of valueLabel
		fData = new FormData();
		fData.top = new FormAttachment(keyCombo);
		fData.width = 50;
		valueLabel.setLayoutData(fData);
		
		
		// Layout of valueText
		fData = new FormData();
		fData.top = new FormAttachment(keyCombo);
		fData.left = new FormAttachment(valueLabel);
		fData.width = 200;
		valueCombo.setLayoutData(fData);		

		// Layout of weightLabel
		fData = new FormData();
		fData.top = new FormAttachment(valueCombo);
		fData.width = 50;
		weightLabel.setLayoutData(fData);

		
		//Layout of weightText
		fData = new FormData();
		fData.top = new FormAttachment(valueCombo);
		fData.left = new FormAttachment(weightLabel);
		fData.width = 200;
		weightText.setLayoutData(fData);	
		
		return cDataComp;
	}
	
	/**
	 * Creates the left side which will display the role tree and has the
	 * buttons to add/remove roles and characteristics
	 * @param parent
	 */
	private Composite createLeftSide(Composite parent) {
		Group left = new Group(parent, SWT.NONE);
		left.setText("Roles and Capabilities");
		left.setLayout(new GridLayout(2,false));
	
		// Add the roleViewer
		roleViewer = new TreeViewer(left);
		GridData gData = new GridData(SWT.FILL,SWT.FILL,true,true);
		gData.widthHint = 180;
		gData.heightHint = 300;
		roleViewer.getTree().setLayoutData(gData);
		roleViewer.setContentProvider(new RoleContentProvider());
		roleViewer.setLabelProvider(new RoleLabelProvider());
		
		roleViewer.addSelectionChangedListener(getEditController());
//		roleViewer.getTree().addListener(SWT.DefaultSelection, getEditController());
		
		// Add the buttons
		intializeButtonsContainer(left);
		
		return left;
	}
	
	private void handleSelectionChange(Object source){
		if(source.equals(getRoleViewer())){
			currentSelection = ((IStructuredSelection)getRoleViewer().getSelection()).getFirstElement();
			if(currentSelection instanceof Role){
				addCharButton.setEnabled(true);
				displayComposite(getRoleDataGroup());
			}
			else if(currentSelection instanceof Characteristic){
				addCharButton.setEnabled(false);
				displayComposite(getCharacterDataGroup());
			}
			else{
				addCharButton.setEnabled(false);
				displayComposite(getNothingToDisplayGroup());
			}
			getEditController().updateView(null);
		}
	}
	
	private TreeViewer getRoleViewer() {
		return roleViewer;
	}
	
	@Override
	public void okPressed() {
		if(roleNameText.isFocusControl() || weightText.isFocusControl() || keyCombo.isFocusControl() || valueCombo.isFocusControl()){
			RoleDefinitionDialog.this.handleModifyEvent();
			shell.setFocus();
		}else{
			close();
		}
	}

	private EditController getEditController() {
		if(editController == null){
			editController = new EditController(){
				// Just delegate all update methods from the controller to
				// methods from the dialog.
				@Override
				protected void doUpdateView(Notification n) {
					RoleDefinitionDialog.this.doUpdateView(n);
				}

				@Override
				protected void enterPressed(Widget source) {
					RoleDefinitionDialog.this.handleModifyEvent();
				}

				@Override
				protected void focusOutEvent(Widget source) {
					RoleDefinitionDialog.this.handleModifyEvent();
				}

				@Override
				protected void modifyEvent(Widget source) {
					// TODO Auto-generated method stub
					
				}

				@Override
				protected void selectionEvent(Object source) {
					handleSelectionChange(source);
				}
			};
		}
		return editController;
	}
	
	private void doUpdateView(Notification n){
		if(currentSelection instanceof Role){
			refreshRoleGroup();
		}else if(currentSelection instanceof Characteristic){
			refreshCharacteristicGroup();
		}
		
		getRoleViewer().refresh(true);
	}
	
	private void handleModifyEvent(){
		// Check which composite is visible
		if(getRoleDataGroup().isVisible()){
			applyRoleData();
		}else if(getCharacterDataGroup().isVisible()){
			applyCharacteristicsData();
		}
	}

	private void applyCharacteristicsData() {
		final Characteristic chara = (Characteristic)currentSelection;
		domain.getCommandStack().execute(
				new RecordingCommand(domain){
					@Override
					protected void doExecute() {
						//chara.setName(keyCombo.getText());
						//chara.setCapid(Long.parseLong(keyCombo.getData(keyCombo.getText()).toString()));
						//chara.setKey(keyCombo.getText());
						//chara.setName(chara.getKey());
						//chara.setValue(valueCombo.getText());
						chara.setCapability((Capability)keyCombo.getData(keyCombo.getText()));
						chara.setName(keyCombo.getText());				
						chara.setValue((CapValue)valueCombo.getData(valueCombo.getText()));
						 
						try {
							double x = Double.parseDouble(weightText.getText());
							chara.setWeight(x);
						} catch (NumberFormatException nm) {
							weightText.setText(((Double)chara.getWeight()).toString());
						}
					}
				});
	}

	private void applyRoleData() {
		Role r = (Role)currentSelection;
		domain.getCommandStack().execute(
				SetCommand.create(
						domain, 
						r, 
						AlicaPackage.eINSTANCE.getPlanElement_Name(), 
						roleNameText.getText()));
	}
}
