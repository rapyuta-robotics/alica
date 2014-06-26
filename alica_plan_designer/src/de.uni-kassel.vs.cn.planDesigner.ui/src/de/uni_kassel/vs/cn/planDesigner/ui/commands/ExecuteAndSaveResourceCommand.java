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
/**
 * 
 */
package de.uni_kassel.vs.cn.planDesigner.ui.commands;

import java.io.IOException;
import java.util.Map;

import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.ecore.resource.Resource;

/**
 * @author till
 *
 */
public class ExecuteAndSaveResourceCommand extends AbstractCommand {

	private Command command;
	
	private Resource resource;
	
	private Map<?, ?> resourceSaveOptions;
	
	public ExecuteAndSaveResourceCommand(Command command, Resource resource, Map<?, ?> resourceSaveOptions) {
		this.command = command;
		this.resource = resource;
		this.resourceSaveOptions = resourceSaveOptions;
	}
	
	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#execute()
	 */
	public void execute() {
		command.execute();
		try {
			resource.save(this.resourceSaveOptions);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/* (non-Javadoc)
	 * @see org.eclipse.emf.common.command.Command#redo()
	 */
	public void redo() {
		execute();
	}
	
	@Override
	public void undo() {
		command.undo();		
		try {
			resource.save(this.resourceSaveOptions);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	@Override
	public boolean canExecute() {
		return command.canExecute();
	}

}
