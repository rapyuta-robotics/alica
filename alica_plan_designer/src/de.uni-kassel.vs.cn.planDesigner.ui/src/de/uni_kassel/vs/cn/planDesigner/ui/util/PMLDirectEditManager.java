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
import org.eclipse.gef.tools.DirectEditManager;
import org.eclipse.swt.widgets.Text;

import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;

public class PMLDirectEditManager extends DirectEditManager {
	
	public PMLDirectEditManager(GraphicalEditPart source, Class editorType, CellEditorLocator locator) {
		super(source,editorType,locator);
	}

	@Override
	protected void initCellEditor() {
		getEditPart().getFigure().validate();
		PlanElement model = (PlanElement)getEditPart().getModel();
		String initialLabelText = model.getName();
		getCellEditor().setValue(initialLabelText);
		Text text = (Text) getCellEditor().getControl();
		text.selectAll();
	}

}
