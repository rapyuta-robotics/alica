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

import java.util.Collections;

import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.DeleteCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.RunnableWithResult;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Widget;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Rating;

public class RatingSection extends PMLPropertySection {

	private Button enabledButton;
	private Button disabledButton;
	private Text nameText;
	private Label nameLabel;

	@Override
	public void createControls(Composite parent,
			TabbedPropertySheetPage tabbedPropertySheetPage) {
		super.createControls(parent, tabbedPropertySheetPage);

		Group group = getWidgetFactory().createGroup(parent, "Rating");
		group.setLayout(new FillLayout());
		Composite form = getWidgetFactory().createFlatFormComposite(group);

		enabledButton = getWidgetFactory().createButton(form, "Enabled",
				SWT.RADIO);
		disabledButton = getWidgetFactory().createButton(form, "Disabled",
				SWT.RADIO);

		nameLabel = getWidgetFactory().createLabel(form, "Name:");
		nameText = getWidgetFactory().createText(form, "",
				SWT.BORDER | SWT.SINGLE);

		// Do the layout
		FormData data = null;

		data = new FormData();
		data.top = new FormAttachment(0, 0);
		data.left = new FormAttachment(0, 0);
		enabledButton.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(0, 0);
		data.left = new FormAttachment(enabledButton, 0);
		disabledButton.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(enabledButton, 0);
		data.left = new FormAttachment(0, 0);
		nameLabel.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(enabledButton, 0);
		data.left = new FormAttachment(enabledButton, 0);
		data.right = new FormAttachment(50, 0);
		nameText.setLayoutData(data);

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
		
		// Add an adapter to the rating if possible
		if(getRating() != null)
			getEditController().addToObject(getRating());
	}
	
	private Rating getRating(){
		return ((AbstractPlan)getModel()).getRating();
	}

	private void registerListeners() {
		getEnabledButton().addListener(SWT.Selection, getEditController());
		getNameText().addListener(SWT.KeyDown, getEditController());
		getNameText().addListener(SWT.FocusOut, getEditController());
	}

	protected Button getEnabledButton() {
		return enabledButton;
	}

	protected Button getDisabledButton() {
		return disabledButton;
	}

	protected Text getNameText() {
		return nameText;
	}

	protected Label getNameLabel() {
		return nameLabel;
	}

	/**
	 * Enabled or disables all visuals
	 * 
	 * @param enabled
	 */
	private void refreshControls() {
		boolean editable = isEditable();
		
		getEnabledButton().setEnabled(editable);
		getDisabledButton().setEnabled(editable);
		
		Rating r = getRating();
		boolean enabledRadioSelected = r != null;
		
		getEnabledButton().setSelection(enabledRadioSelected);
		getDisabledButton().setSelection(!enabledRadioSelected);
		
		getNameLabel().setEnabled(editable);
		getNameText().setEnabled(editable && enabledRadioSelected);
	}
	
	@Override
	protected void updateView(Notification n){
		refreshControls();
		
		final Rating rating = getRating();
		if(rating != null){
			try {
				String name = (String)getEditingDomain().runExclusive(new RunnableWithResult.Impl<String>(){
					public void run() {
						setResult(rating.getName());
					}
				});
				getNameText().setText(name);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
		}
		
	}
	
	protected void resetUI(){
		getNameText().setText("");
	}
	
	@Override
	public void refresh() {
		getEditController().updateView(null);
	}
	
	@Override
	protected void selectionEvent(Object source) {
		if(source.equals(getEnabledButton())){
			boolean enabled = getEnabledButton().getSelection();
			
//			enableVisuals(enabled);

			CompoundCommand cmd = new CompoundCommand(0);

			if (enabled) {
				Rating rating = getRating();
				if (rating == null) {
					// Create a new Rating and initialize it's values with
					// those from the textFields
					rating = AlicaFactory.eINSTANCE.createRating();
					
					cmd.append(CreateChildCommand.create(
							getEditingDomain(), 
							getModel(), 
							new CommandParameter(null,null,rating), 
							Collections.EMPTY_LIST));
					
					// Add the model to the editController
					getEditController().addToObject(rating);
				}
				
				// Try to apply values, already given in the fields
				cmd.append(SetCommand.create(
						getEditingDomain(), 
						rating, 
						AlicaPackage.eINSTANCE.getPlanElement_Name(), 
						getNameText().getText()));
				

			} else {
				cmd.append(DeleteCommand.create(getEditingDomain(), Collections.singleton(getRating())));
				
				// Remove the model from the editController
				getEditController().removeFromObject(getRating());
			}

			executeCommand(cmd);

		}
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
			if (getRating() != null)
				executeCommand(SetCommand.create(
						getEditingDomain(), 
						getRating(), 
						AlicaPackage.eINSTANCE.getPlanElement_Name(), 
						getNameText().getText()));
		}
	}
}
