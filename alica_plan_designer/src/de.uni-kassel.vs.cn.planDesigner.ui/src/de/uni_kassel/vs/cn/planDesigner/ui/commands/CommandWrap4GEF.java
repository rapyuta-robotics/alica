/**
 * Copyright (c) 2002-2003, Eric Suen and Contributors.
 * All rights reserved.   This program and the accompanying materials
 * are made available under the terms of the Common Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/cpl-v10.html
 * 
 * Contributors: 
 *   Eric Suen - Initial API and implementation
 *
 * $Id$
 * 
 * Changes
 * -------
 * 
 */
package de.uni_kassel.vs.cn.planDesigner.ui.commands;

import java.util.Collection;
import java.util.Collections;

import org.eclipse.emf.common.command.Command;

/**
 *
 */
public class CommandWrap4GEF implements Command {
	private org.eclipse.gef.commands.Command command;
	
	/**
	 * Constructor for CommandWrap4GEF.
	 */
	public CommandWrap4GEF(org.eclipse.gef.commands.Command command) {
		this.command = command;
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#canExecute()
	 */
	public boolean canExecute() {
		return command == null ? false : command.canExecute();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#canUndo()
	 */
	public boolean canUndo() {
		return command == null ? false : command.canUndo();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#chain(Command)
	 */
	public Command chain(Command command) {
		Command chained = this;
		if (command != null) {
			org.eclipse.gef.commands.Command gefCommand = null;
			if (command instanceof CommandWrap4GEF) {
				gefCommand = ((CommandWrap4GEF) command).unwrap();
			} else {
				gefCommand = new CommandWrap4EMF(command);
			}
			gefCommand = this.command.chain(gefCommand);
			if (gefCommand != null) {
				if (gefCommand instanceof CommandWrap4EMF) {
					chained = ((CommandWrap4EMF) gefCommand).unwrap();
				} else {
					chained = new CommandWrap4GEF(gefCommand);
				}
			}
		}
		return chained;
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#dispose()
	 */
	public void dispose() {
		if (command != null) command.dispose();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#execute()
	 */
	public void execute() {
		if (command != null) command.execute();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#getAffectedObjects()
	 */
	public Collection getAffectedObjects() {
		return Collections.EMPTY_LIST;
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#getDescription()
	 */
	public String getDescription() {
		return "CommandWrap4GEF"; //$NON-NLS-1$
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#getLabel()
	 */
	public String getLabel() {
		return command == null ? "CommandWrap4GEF" : command.getLabel(); //$NON-NLS-1$
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#getResult()
	 */
	public Collection getResult() {
		return Collections.EMPTY_LIST;
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#redo()
	 */
	public void redo() {
		if (command != null) command.redo();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#undo()
	 */
	public void undo() {
		if (command != null) command.undo();
	}

	public org.eclipse.gef.commands.Command unwrap() {
		return command;
	}
}
