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
package de.uni_kassel.vs.cn.planDesigner.ui.policies;

import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.editpolicies.DirectEditPolicy;
import org.eclipse.gef.requests.DirectEditRequest;
import org.eclipse.jface.viewers.CellEditor;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CommandWrap4EMF;
import de.uni_kassel.vs.cn.planDesigner.ui.util.RolesetEditorUtils;

public class TaskPriorityDirectEditPolicy extends DirectEditPolicy {

	@Override
	protected Command getDirectEditCommand(DirectEditRequest request) {
		Command cmd = null;
		CellEditor cellEditor = request.getCellEditor();
		if (cellEditor == null || !cellEditor.isValueValid()) {
			return cmd;
		}
		// At this point we are sure that the value is valid
		// due to celleditor valid checks
		double priority = Double.parseDouble(((String)cellEditor.getValue()).replaceAll(",","."));
		InternalRoleTaskMapping mapping = (InternalRoleTaskMapping)getHost().getModel();
		TransactionalEditingDomain editingDomain = RolesetEditorUtils.getEditingDomainAdapter(getHost());
		
		org.eclipse.emf.common.command.Command emfCmd =  
								SetCommand.create(
										editingDomain, 
										mapping, 
										AlicaPackage.eINSTANCE.getInternalRoleTaskMapping_Priority(), 
										priority);

		cmd = new CommandWrap4EMF(emfCmd);
		
		return cmd;
	}

	@Override
	protected void showCurrentEditValue(DirectEditRequest request) {
		// Don't do anything at the moment
	}

	
}
