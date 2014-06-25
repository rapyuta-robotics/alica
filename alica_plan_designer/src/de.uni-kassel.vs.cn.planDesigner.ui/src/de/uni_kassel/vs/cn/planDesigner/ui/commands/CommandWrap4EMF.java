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

import org.eclipse.gef.commands.Command;

/**
 *
 */
public class CommandWrap4EMF extends Command {
	private org.eclipse.emf.common.command.Command command;

	/**
	 * Constructor for CommandWrap4EMF.
	 */
	public CommandWrap4EMF(org.eclipse.emf.common.command.Command command) {
		this.command = command;
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#canExecute()
	 */
	public boolean canExecute() {
		return command == null ? false : command.canExecute();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#canUndo()
	 */
	public boolean canUndo() {
		return command == null ? false : command.canUndo();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#chain(Command)
	 */
	public Command chain(Command command) {
		Command chained = this;
		if (command != null) {
			org.eclipse.emf.common.command.Command emfCommand = null;
			if (command instanceof CommandWrap4EMF) {
				emfCommand = ((CommandWrap4EMF) command).unwrap();
			} else {
				emfCommand = new CommandWrap4GEF(command);
			}
			emfCommand = this.command.chain(emfCommand);
			if (emfCommand != null) {
				if (emfCommand instanceof CommandWrap4GEF) {
					chained = ((CommandWrap4GEF) emfCommand).unwrap();
				} else {
					chained = new CommandWrap4EMF(emfCommand);
				}
			}
		}
		return chained;
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#dispose()
	 */
	public void dispose() {
		if (command != null) command.dispose();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#execute()
	 */
	public void execute() {
		if (command != null) command.execute();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#getLabel()
	 */
	public String getLabel() {
		return command == null ? "CommandWrap4EMF" : command.getLabel(); //$NON-NLS-1$
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#redo()
	 */
	public void redo() {
		if (command != null) command.redo();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#undo()
	 */
	public void undo() {
		if (command != null) command.undo();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#getAffectedObjects()
	 */
	public Collection getAffectedObjects() {
		return command == null ? Collections.EMPTY_LIST : command.getAffectedObjects();
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#getDescription()
	 */
	public String getDescription() {
		return command == null ? "CommandWrap4EMF" : command.getDescription(); //$NON-NLS-1$
	}

	/* (non-Javadoc)
	 * @see org.eclipse.gef.commands.Command#getResult()
	 */
	public Collection getResult() {
		return command == null ? Collections.EMPTY_LIST : command.getResult();
	}
	
	public org.eclipse.emf.common.command.Command unwrap() {
		return command;
	}

}
