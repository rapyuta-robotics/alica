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

import java.util.EventObject;
import java.util.Iterator;

import org.eclipse.core.commands.operations.IOperationHistory;
import org.eclipse.emf.workspace.impl.WorkspaceCommandStackImpl;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.commands.CommandStack;
import org.eclipse.gef.commands.CommandStackListener;

/**
 * This class is a way to combine to two world: EMF and GEF. Both operate on CommandStack and 
 * Command interfaces which are very similar but to get the advantage of EMF Commands we have to
 * provide our own commandStack which has an inner GEF command stack.
 * @author Zenobios
 *
 */
public class EMF2GEFCommandStack extends WorkspaceCommandStackImpl {
	
	public EMF2GEFCommandStack(IOperationHistory history) {
		super(history);
	}

	/**
	 * This is the inner GEFCommandStack which is used within the GEF Framework. 
	 * @author Zenobios
	 *
	 */
	private class GEFCommandStack extends CommandStack {

		
		public boolean canRedo() {
			return EMF2GEFCommandStack.this.canRedo();
		}

		@Override
		public boolean canUndo() {
			return EMF2GEFCommandStack.this.canUndo();
		}

		@Override
		public void execute(Command command) {
			if (command != null) {
				org.eclipse.emf.common.command.Command emfCommand = null;
				if (command instanceof CommandWrap4EMF) {
					emfCommand = ((CommandWrap4EMF) command).unwrap();
				} else {
					emfCommand = new CommandWrap4GEF(command);
				}
				EMF2GEFCommandStack.this.execute(emfCommand);
			}
		}

		@Override
		public void flush() {
			EMF2GEFCommandStack.this.flush();
		}

		@Override
		public Command getRedoCommand() {
			org.eclipse.emf.common.command.Command command = EMF2GEFCommandStack.this.getRedoCommand();
			Command gefCommand = null;
			if (command != null) {
				if (command instanceof CommandWrap4GEF) {
					gefCommand = ((CommandWrap4GEF) command).unwrap();
				} else {
					gefCommand = new CommandWrap4EMF(command);
				}
			}
			return gefCommand;
		}

		@Override
		public Command getUndoCommand() {
			org.eclipse.emf.common.command.Command command = EMF2GEFCommandStack.this.getUndoCommand();
			Command gefCommand = null;
			if(command != null){
				if (command instanceof CommandWrap4GEF) {
					gefCommand = ((CommandWrap4GEF) command).unwrap();
				} else {
					gefCommand = new CommandWrap4EMF(command);
				}
			}
			
			return gefCommand;
		}

		@Override
		public void redo() {
			EMF2GEFCommandStack.this.redo();
		}

		@Override
		public void undo() {
			EMF2GEFCommandStack.this.undo();
		}
		
		@Override
		public boolean isDirty() {
			return EMF2GEFCommandStack.this.isSaveNeeded();
		}
	}
	
	private GEFCommandStack gefCommandStack = new GEFCommandStack();
	
	public CommandStack getCommandStack4GEF() {
		return gefCommandStack;
	}
	
	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.BasicCommandStack#notifyListeners()
	 */
	protected void notifyListeners() {
		for (Iterator i = listeners.iterator(); i.hasNext();) {
			Object listener = i.next();
			if (listener instanceof org.eclipse.emf.common.command.CommandStackListener) {
				((org.eclipse.emf.common.command.CommandStackListener) listener).commandStackChanged(new EventObject(this));
			} else {
				((CommandStackListener) listener).commandStackChanged(new EventObject(gefCommandStack));
			}
		}
	}
	
	@Override
	public void saveIsDone() {
		super.saveIsDone();
		notifyListeners();
	}

}
