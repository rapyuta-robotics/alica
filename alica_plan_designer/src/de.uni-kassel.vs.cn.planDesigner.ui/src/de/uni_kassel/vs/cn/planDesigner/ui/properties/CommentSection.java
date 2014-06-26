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
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Widget;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;

public class CommentSection extends PMLPropertySection {

	private Text commentText;
	private Label commentLabel;

	@Override
	public void createControls(Composite parent,
			TabbedPropertySheetPage tabbedPropertySheetPage) {
		super.createControls(parent, tabbedPropertySheetPage);
		
		Composite form = getWidgetFactory().createFlatFormComposite(parent);
		
		commentLabel = getWidgetFactory().createLabel(form, "Comment:");
		commentText = getWidgetFactory().createText(form, "", SWT.BORDER | SWT.MULTI | SWT.WRAP | SWT.V_SCROLL);
		
		// Do the layout
		FormData data = null;

		data = new FormData();
		data.top = new FormAttachment(0, 0);
		data.left = new FormAttachment(0, 0);
		commentLabel.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(0,0);
		data.left = new FormAttachment(commentLabel);
		data.right = new FormAttachment(50, 0);
		data.height = 200;
		commentText.setLayoutData(data);
		

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
	
	private PlanElement getPlanElement(){
		return ((PlanElement)getModel());
	}

	private void registerListeners() {
//		getCommentText().addListener(SWT.KeyDown, getEditController());
//		getCommentText().addListener(SWT.FocusOut, getEditController());
		getCommentText().addListener(SWT.Modify, getEditController());
	}

	protected Text getCommentText() {
		return commentText;
	}

	protected Label getNameLabel() {
		return commentLabel;
	}

	@Override
	protected void updateView(Notification n) {
		refreshControls();
		int pos = getCommentText().getCaretPosition();
		getCommentText().setText(getPlanElement().getComment());
		getCommentText().setSelection(pos);
	}
	
	private void refreshControls(){
		getCommentLabel().setEnabled(isEditable());
		getCommentText().setEnabled(isEditable());
	}
	
	protected void resetUI(){
		getCommentText().setText("");
	}
	
	@Override
	public void refresh() {
		getEditController().updateView(null);
	}
	
	@Override
	protected void modifyEvent(Widget source) {
		applyValueToModel(source);
	}
	
	private void applyValueToModel(Widget source){
		if(source.equals(getCommentText())){
			executeCommand(SetCommand.create(
					getEditingDomain(), 
					getPlanElement(), 
					AlicaPackage.eINSTANCE.getPlanElement_Comment(), 
					getCommentText().getText()));
		}
	}

	private Label getCommentLabel() {
		return commentLabel;
	}
}
