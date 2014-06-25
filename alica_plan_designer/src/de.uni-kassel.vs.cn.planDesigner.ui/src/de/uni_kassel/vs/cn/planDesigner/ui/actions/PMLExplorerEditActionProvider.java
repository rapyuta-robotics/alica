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

import org.eclipse.jface.action.IMenuManager;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.swt.dnd.Clipboard;
import org.eclipse.ui.IActionBars;
import org.eclipse.ui.ISharedImages;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.actions.ActionFactory;
import org.eclipse.ui.navigator.CommonActionProvider;
import org.eclipse.ui.navigator.ICommonActionExtensionSite;

public class PMLExplorerEditActionProvider extends CommonActionProvider {
	
	private PastePlanAction pastePlanAction;
	private CopyPlanAction copyPlanAction;
	private ICommonActionExtensionSite site;
	
//	private MyEditActionGroup editActionGroup;

	public PMLExplorerEditActionProvider() {
	}
	
	@Override
	public void init(ICommonActionExtensionSite site) {
		super.init(site);
		this.site = site;
//		this.editActionGroup = new MyEditActionGroup(site.getViewSite().getShell());
		makeActions();
	}
	
	private void makeActions(){
		Clipboard clipboard = new Clipboard(site.getViewSite().getShell().getDisplay());
		
		ISharedImages images = PlatformUI.getWorkbench().getSharedImages();

		pastePlanAction = new PastePlanAction(site.getViewSite().getShell(), clipboard);
		pastePlanAction.setImageDescriptor(images.getImageDescriptor(ISharedImages.IMG_TOOL_PASTE));
		pastePlanAction.setDisabledImageDescriptor(images.getImageDescriptor(ISharedImages.IMG_TOOL_PASTE));
		
		copyPlanAction = new CopyPlanAction(site.getViewSite().getShell(), clipboard, pastePlanAction);
		copyPlanAction.setImageDescriptor(images.getImageDescriptor(ISharedImages.IMG_TOOL_COPY));
		copyPlanAction.setDisabledImageDescriptor(images.getImageDescriptor(ISharedImages.IMG_TOOL_COPY));
		
	}

	@Override
	public void fillActionBars(IActionBars actionBars) { 
		actionBars.setGlobalActionHandler(ActionFactory.PASTE.getId(), pastePlanAction);
		
		updateActionBars();
		
		actionBars.updateActionBars();
	}
	
	@Override
	public void fillContextMenu(IMenuManager menu) {
		menu.remove(PlatformUI.PLUGIN_ID + ".PasteAction");
		menu.insertAfter(PlatformUI.PLUGIN_ID + ".CopyAction", pastePlanAction);
		updateActionBars();
	}
	
	@Override
	public void updateActionBars() {
		IStructuredSelection selection = (IStructuredSelection)getContext().getSelection();
		
		pastePlanAction.selectionChanged(selection);
	}
}
