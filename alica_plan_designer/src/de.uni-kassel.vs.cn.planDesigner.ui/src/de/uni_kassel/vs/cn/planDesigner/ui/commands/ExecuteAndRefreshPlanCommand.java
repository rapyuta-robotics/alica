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

import java.util.List;

import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.common.command.Command;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.GraphicalEditPart;

public class ExecuteAndRefreshPlanCommand extends AbstractCommand
{

	private final Command cmd;
	private final GraphicalEditPart rootEditPart;

	public ExecuteAndRefreshPlanCommand(Command cmd, GraphicalEditPart rootEditPart)
	{
		super(cmd.getLabel());
		this.cmd = cmd;
		this.rootEditPart = rootEditPart;
	}
	
	public void execute()
	{
		cmd.execute();
		refreshPlan();
	}
	
	private void refreshPlan()
	{
		visit(rootEditPart);
	}
	
	private void visit(EditPart part)
	{
		part.refresh();
		List<EditPart> children = part.getChildren();
		for(EditPart child : children)
		{
			visit(child);
		}
	}

	public void redo()
	{
		cmd.redo();
		refreshPlan();
	}
	
	@Override
	public boolean canExecute()
	{
		return cmd.canExecute();
	}
	
	@Override
	public void undo()
	{
		cmd.undo();
		refreshPlan();
	}
	
	@Override
	public boolean canUndo()
	{
		return cmd.canUndo();
	}
}
