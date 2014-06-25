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
package de.uni_kassel.vs.cn.planDesigner.ui.properties;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Widget;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;

public class TaskSection extends PMLPropertySection {

	private Text nameText;
	private Text descriptionText;
	private Label nameLabel;
	private Label descriptionLabel;

	@Override
	public void createControls(Composite parent,
			TabbedPropertySheetPage tabbedPropertySheetPage) {
		super.createControls(parent, tabbedPropertySheetPage);

		Group group = getWidgetFactory().createGroup(parent, "Task");
		group.setLayout(new FillLayout());
		Composite form = getWidgetFactory().createFlatFormComposite(group);

		nameLabel = getWidgetFactory().createLabel(form, "Name:");
		nameText = getWidgetFactory().createText(form, "",
				SWT.BORDER | SWT.SINGLE);

		descriptionLabel = getWidgetFactory().createLabel(form, "Description:");
		descriptionText = getWidgetFactory().createText(form, "",
				SWT.BORDER | SWT.SINGLE);

		// Do the layout
		FormData data = null;

		data = new FormData();
		data.top = new FormAttachment(0, 0);
		data.left = new FormAttachment(0, 29);
		nameLabel.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(0, 0);
		data.left = new FormAttachment(nameLabel, 0);
		data.right = new FormAttachment(50, 0);
		nameText.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(nameText, 0);
		data.left = new FormAttachment(0, 0);
		descriptionLabel.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(nameText, 0);
		data.left = new FormAttachment(descriptionLabel, 0);
		data.right = new FormAttachment(50, 0);
		descriptionText.setLayoutData(data);

		registerListeners();
	}

	@Override
	protected void basicSetInput(Object input) {
		super.basicSetInput(input);

		// Reset the UI
		getEditController().pauseListening();
		resetUI();
		getEditController().resumeListening();
		
	}
	
	@Override
	protected void addAllAdapters() {
		super.addAllAdapters();
		
		// Add an adapter to the task if possible
		if(getTask() != null)
			getEditController().addToObject(getTask());
	}
	
	private Task getTask(){
		return ((EntryPoint)getModel()).getTask();
	}

	private void registerListeners() {
		getNameText().addListener(SWT.KeyDown, getEditController());
		getNameText().addListener(SWT.FocusOut, getEditController());
		getDescriptionText().addListener(SWT.KeyDown, getEditController());
		getDescriptionText().addListener(SWT.FocusOut, getEditController());
	}

	protected Text getNameText() {
		return nameText;
	}

	protected Text getDescriptionText() {
		return descriptionText;
	}

	protected Label getNameLabel() {
		return nameLabel;
	}

	protected Label getConditionLabel() {
		return descriptionLabel;
	}

	@Override
	protected void updateView(Notification n) {
		Task task = getTask();
		
		refreshControls();
		
		getNameText().setText(task.getName());
		getDescriptionText().setText(task.getDescription());
	}
	
	private void refreshControls(){
		boolean editable = isEditable();
		
		getNameText().setEnabled(editable);
		getDescriptionText().setEnabled(editable);
	}
	
	protected void resetUI(){
		getNameText().setText("");
		getDescriptionText().setText("");
	}
	
	@Override
	public void refresh() {
		getEditController().updateView(null);
	}
	
	@Override
	protected void focusOutEvent(Widget source) {
		applyValueToModel(source);
	}
	
	@Override
	protected void enterPressed(Widget source) {
		// Save caret position
		int pos = ((Text)source).getCaretPosition();
		applyValueToModel(source);
		// Apply the caret position
		((Text)source).setSelection(pos);
	}
	
	private void applyValueToModel(Widget source){
		if(source.equals(getNameText())){
			if (getTask() != null)
				executeCommand(SetCommand.create(
						getEditingDomain(), 
						getTask(), 
						AlicaPackage.eINSTANCE.getPlanElement_Name(), 
						getNameText().getText()));
		}else if(source.equals(getDescriptionText())){
			if (getTask() != null)
				executeCommand(SetCommand.create(
						getEditingDomain(), 
						getTask(), 
						AlicaPackage.eINSTANCE.getTask_Description(), 
						getDescriptionText().getText()));
		}
	}
}
