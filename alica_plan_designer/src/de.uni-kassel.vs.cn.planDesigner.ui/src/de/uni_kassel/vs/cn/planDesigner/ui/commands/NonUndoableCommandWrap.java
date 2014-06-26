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

/**
 * This command wraps any command which doesn't want to be undoable. 
 * A common case for this are operations which have to be executed through
 * the command stack, but shouldn't appear in the undo history. 
 * @author Zenobios
 *
 */
public class NonUndoableCommandWrap extends AbstractCommand {
	
	private Command cmd;

	public NonUndoableCommandWrap(Command commandToExecute){
		super(commandToExecute.getLabel());
		this.cmd = commandToExecute;
	}

	public void execute() {
		cmd.execute();
	}

	public void redo() {
		// Nothing to do, since we can't undo
	}
	
	@Override
	public boolean canExecute() {
		return cmd.canExecute();
	}
	
	@Override
	public boolean canUndo() {
		return false;
	}

}
