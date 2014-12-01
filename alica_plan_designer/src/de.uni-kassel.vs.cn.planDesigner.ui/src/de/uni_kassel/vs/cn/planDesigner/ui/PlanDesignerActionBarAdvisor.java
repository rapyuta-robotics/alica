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
package de.uni_kassel.vs.cn.planDesigner.ui;

import org.eclipse.jface.action.GroupMarker;
import org.eclipse.jface.action.IContributionItem;
import org.eclipse.jface.action.ICoolBarManager;
import org.eclipse.jface.action.IMenuManager;
import org.eclipse.jface.action.IStatusLineManager;
import org.eclipse.jface.action.MenuManager;
import org.eclipse.jface.action.Separator;
import org.eclipse.jface.action.ToolBarManager;
import org.eclipse.ui.IWorkbenchActionConstants;
import org.eclipse.ui.IWorkbenchWindow;
import org.eclipse.ui.actions.ActionFactory;
import org.eclipse.ui.actions.ContributionItemFactory;
import org.eclipse.ui.application.ActionBarAdvisor;
import org.eclipse.ui.application.IActionBarConfigurer;

public class PlanDesignerActionBarAdvisor extends ActionBarAdvisor {
	private IContributionItem perspShortList;
	
	public PlanDesignerActionBarAdvisor(IActionBarConfigurer configurer) {
		super(configurer);
	}
	
	@Override
	protected void makeActions(IWorkbenchWindow window) {
		register(ActionFactory.QUIT.create(window));
		register(ActionFactory.UNDO.create(window));
		register(ActionFactory.REDO.create(window));
		register(ActionFactory.CUT.create(window));
		register(ActionFactory.COPY.create(window));
		register(ActionFactory.PASTE.create(window));
		register(ActionFactory.DELETE.create(window));
		register(ActionFactory.SAVE.create(window));
		register(ActionFactory.SAVE_ALL.create(window));
		register(ActionFactory.RESET_PERSPECTIVE.create(window));
		register(ActionFactory.OPEN_PERSPECTIVE_DIALOG.create(window));
		register(ActionFactory.HELP_CONTENTS.create(window));
		register(ActionFactory.PREFERENCES.create(window));
		register(ActionFactory.ABOUT.create(window));
		
		perspShortList = ContributionItemFactory.PERSPECTIVES_SHORTLIST.create(window);
		
	}
	
	@Override
	protected void fillMenuBar(IMenuManager menuBar) {
		// File
		MenuManager fileMenu = new MenuManager("&File", IWorkbenchActionConstants.M_FILE);
		fileMenu.add(new Separator(IWorkbenchActionConstants.FILE_START));
		fileMenu.add(getAction(ActionFactory.SAVE.getId()));
		fileMenu.add(getAction(ActionFactory.SAVE_ALL.getId()));
		fileMenu.add(getAction(ActionFactory.QUIT.getId()));
		fileMenu.add(new Separator(IWorkbenchActionConstants.FILE_END));
		menuBar.add(fileMenu);
		
		// Edit
		MenuManager editMenu = new MenuManager("&Edit", IWorkbenchActionConstants.M_EDIT);
		editMenu.add(new GroupMarker(IWorkbenchActionConstants.FIND_EXT));
		editMenu.add(new Separator(IWorkbenchActionConstants.EDIT_START));
		editMenu.add(getAction(ActionFactory.UNDO.getId()));
		editMenu.add(getAction(ActionFactory.REDO.getId()));
		editMenu.add(new Separator(IWorkbenchActionConstants.EDIT_END));
		editMenu.add(getAction(ActionFactory.CUT.getId()));
		editMenu.add(getAction(ActionFactory.COPY.getId()));
		editMenu.add(getAction(ActionFactory.PASTE.getId()));
		editMenu.add(new Separator());
		editMenu.add(getAction(ActionFactory.DELETE.getId()));
		menuBar.add(editMenu);
		
		// Window
		MenuManager windowMenu = new MenuManager("&Window", IWorkbenchActionConstants.M_WINDOW);
//		windowMenu.add(getAction(ActionFactory.OPEN_PERSPECTIVE_DIALOG.getId()));
		windowMenu.add(getAction(ActionFactory.RESET_PERSPECTIVE.getId()));
		windowMenu.add(getAction(ActionFactory.PREFERENCES.getId()));
		menuBar.add(windowMenu);
		
		menuBar.add(new GroupMarker(IWorkbenchActionConstants.MB_ADDITIONS));
		
		// Help
		MenuManager helpMenu = new MenuManager("&Help", IWorkbenchActionConstants.M_HELP);
		helpMenu.add(new Separator(IWorkbenchActionConstants.HELP_START));
		helpMenu.add(getAction(ActionFactory.HELP_CONTENTS.getId()));
		// For update actions
		helpMenu.add(new Separator("update"));
		helpMenu.add(new Separator(IWorkbenchActionConstants.HELP_END));
		helpMenu.add(getAction(ActionFactory.ABOUT.getId()));
		menuBar.add(helpMenu);
	}
	
	@Override
	protected void fillCoolBar(ICoolBarManager coolBar) {
		// File Tools
		ToolBarManager fileTools = new ToolBarManager();
		fileTools.add(getAction(ActionFactory.SAVE.getId()));
		fileTools.add(getAction(ActionFactory.SAVE_ALL.getId()));
		coolBar.add(fileTools);
//		coolBar.add(new GroupMarker(IWorkbenchActionConstants.MB_ADDITIONS));
		
	}
	
	@Override
	protected void fillStatusLine(IStatusLineManager statusLine) {
		super.fillStatusLine(statusLine);
	}

}
