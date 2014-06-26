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
package de.uni_kassel.vs.cn.planDesigner.ui.actions;

import java.util.Map;

import org.eclipse.gef.editparts.AbstractGraphicalEditPart;
import org.eclipse.gef.ui.actions.WorkbenchPartAction;
import org.eclipse.ui.IWorkbenchPart;

import de.uni_kassel.vs.cn.planDesigner.ui.editors.RolesetEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.TaskGraphEditPart;

public class ExpandCollapseAllAction extends WorkbenchPartAction {
	
	public static final String ID_COLLAPSE = "collapseAllAction";
	public static final String ID_EXPAND = "expandAllAction";
	private boolean collapse;
	

	public ExpandCollapseAllAction(IWorkbenchPart part, boolean collapse) {
		super(part);
		setId(collapse ? ID_COLLAPSE : ID_EXPAND);
		setText((collapse ? "Collapse" : "Expand") +" all");
		this.collapse = collapse;
	}
	
	@Override
	public void run() {
		RolesetEditor editor = (RolesetEditor)getWorkbenchPart();
		Map<Object, AbstractGraphicalEditPart> registry = editor.getGraphicalViewer().getEditPartRegistry();
		TaskGraphEditPart diagram = (TaskGraphEditPart)registry.get(editor.getTaskGraph());
		
		// Tell the diagram to auto-layout
		diagram.expandOrCollapseAll(collapse);
	}


	@Override
	protected boolean calculateEnabled() {
		return true;
	}
}
