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
package de.uni_kassel.vs.cn.planDesigner.ui.editors;
import org.eclipse.emf.edit.ui.action.EditingDomainActionBarContributor;
import org.eclipse.emf.workspace.ui.actions.RedoActionWrapper;
import org.eclipse.emf.workspace.ui.actions.UndoActionWrapper;
import org.eclipse.gef.editparts.ZoomManager;
import org.eclipse.gef.ui.actions.ZoomComboContributionItem;
import org.eclipse.jface.action.IToolBarManager;
import org.eclipse.jface.action.Separator;
import org.eclipse.ui.IActionBars;
import org.eclipse.ui.ISharedImages;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.actions.ActionFactory;

import de.uni_kassel.vs.cn.planDesigner.ui.actions.DeleteAction;


public class RolesetEditorActionBarContributor extends EditingDomainActionBarContributor {
	
	@Override
	public void init(IActionBars actionBars) {
		super.init(actionBars);
		
		ISharedImages sharedImages = PlatformUI.getWorkbench().getSharedImages();
	    
		//override the superclass implementation of these actions
	    undoAction = new UndoActionWrapper();
	    undoAction.setImageDescriptor(sharedImages.getImageDescriptor(ISharedImages.IMG_TOOL_UNDO));
	    actionBars.setGlobalActionHandler(ActionFactory.UNDO.getId(), undoAction);

	    redoAction = new RedoActionWrapper();
	    redoAction.setImageDescriptor(sharedImages.getImageDescriptor(ISharedImages.IMG_TOOL_REDO));
	    actionBars.setGlobalActionHandler(ActionFactory.REDO.getId(), redoAction);
	    
	    deleteAction = new DeleteAction();
	    deleteAction.setImageDescriptor(sharedImages.getImageDescriptor(ISharedImages.IMG_TOOL_DELETE));
	    actionBars.setGlobalActionHandler(ActionFactory.DELETE.getId(), deleteAction);
	    
	    actionBars.getToolBarManager().add(undoAction);
	    actionBars.getToolBarManager().add(redoAction);
	}
	
	@Override
	public void contributeToToolBar(IToolBarManager toolBarManager) {
		// Add Zoom support
		toolBarManager.add(new Separator());
		String[] zoomStrings = new String[] {	ZoomManager.FIT_ALL, 
				ZoomManager.FIT_HEIGHT, 
				ZoomManager.FIT_WIDTH	};
		toolBarManager.add(new ZoomComboContributionItem(getPage(), zoomStrings));
	}
}
