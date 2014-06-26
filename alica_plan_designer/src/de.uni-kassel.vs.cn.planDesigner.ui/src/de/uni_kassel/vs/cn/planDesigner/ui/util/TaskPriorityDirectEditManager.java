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

import org.eclipse.gef.GraphicalEditPart;
import org.eclipse.gef.tools.CellEditorLocator;
import org.eclipse.jface.viewers.ICellEditorValidator;
import org.eclipse.swt.widgets.Text;

import de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping;

public class TaskPriorityDirectEditManager extends PMLDirectEditManager {

	public TaskPriorityDirectEditManager(GraphicalEditPart source,
			Class editorType, CellEditorLocator locator) {
		super(source, editorType, locator);
	}
	
	@Override
	protected void initCellEditor() {
		getEditPart().getFigure().validate();
		
		InternalRoleTaskMapping model = (InternalRoleTaskMapping)getEditPart().getModel();
		double initialLabelText = model.getPriority();
		getCellEditor().setValue(String.valueOf(initialLabelText));
		Text text = (Text) getCellEditor().getControl();
		text.selectAll();
		
		// Add an verifyListener
		getCellEditor().setValidator(new ICellEditorValidator(){

			public String isValid(Object value) {
				try {
					double prio = Double.parseDouble(((String)value).replaceAll(",", "."));
					if(prio < -1.0 || prio > 1.0){
						return "Please enter a value between -1.0 and 1.0";
					}else{
						return null;
					}
				} catch (NumberFormatException e) {
					return "Please enter a double value";
				}
			}
			
		});
	}

}
