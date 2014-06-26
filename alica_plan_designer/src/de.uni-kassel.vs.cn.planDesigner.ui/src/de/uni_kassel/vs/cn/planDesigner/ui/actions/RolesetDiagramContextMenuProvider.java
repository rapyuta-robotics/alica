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

import org.eclipse.gef.ContextMenuProvider;
import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.ui.actions.ActionRegistry;
import org.eclipse.jface.action.IAction;
import org.eclipse.jface.action.IMenuManager;
import org.eclipse.jface.action.Separator;
import org.eclipse.ui.IWorkbenchActionConstants;


public class RolesetDiagramContextMenuProvider extends ContextMenuProvider {

	private ActionRegistry registry;

	public RolesetDiagramContextMenuProvider(EditPartViewer viewer, ActionRegistry registry) {
		super(viewer);
		this.registry = registry;
	}

	@Override
	public void buildContextMenu(IMenuManager menu) {
		IAction action = null;
		menu.add(new Separator("layout"));
		
		action = registry.getAction(AutoLayoutAction.ID);
		if(action.isEnabled())
			menu.add(action);
		
		action = registry.getAction(UpdateRoleSetAction.ID);
		if(action.isEnabled())
			menu.add(action);
		
		action = registry.getAction(ExpandCollapseAllAction.ID_EXPAND);
		if(action.isEnabled())
			menu.add(action);
		
		action = registry.getAction(ExpandCollapseAllAction.ID_COLLAPSE);
		if(action.isEnabled())
			menu.add(action);
		
		menu.add(new Separator(IWorkbenchActionConstants.MB_ADDITIONS));
	}

}
