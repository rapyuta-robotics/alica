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
package de.uni_kassel.vs.cn.planDesigner.ui.commands;

import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.common.command.Command;
import org.eclipse.gef.EditPartViewer;

import de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph;
import de.uni_kassel.vs.cn.planDesigner.ui.util.RolesetEditorUtils;

public class ExecuteAndRefreshGraphCommand extends AbstractCommand {
	
	private EditPartViewer viewer;
	private TaskGraph graph;
	private Command command;
	

	public ExecuteAndRefreshGraphCommand(String label, Command command, EditPartViewer viewer, TaskGraph graph){
		super(label);
		this.viewer = viewer;
		this.graph = graph;
		this.command = command;
	}
	
	public void execute() {
		// Execute the command
		command.execute();
		RolesetEditorUtils.refreshTaskGraphUI(graph, viewer);
	}

	public void redo() {
		command.redo();
		RolesetEditorUtils.refreshTaskGraphUI(graph, viewer);
	}
	
	@Override
	public void undo() {
		command.undo();
		RolesetEditorUtils.refreshTaskGraphUI(graph, viewer);
	}
	
	@Override
	public boolean canExecute() {
		return command.canExecute();
	}
	
	@Override
	protected boolean prepare() {
		return true;
	}

}
