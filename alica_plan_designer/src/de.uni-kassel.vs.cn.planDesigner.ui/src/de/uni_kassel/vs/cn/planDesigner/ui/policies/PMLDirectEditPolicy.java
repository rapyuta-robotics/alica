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
import org.eclipse.emf.transaction.util.TransactionUtil;
import org.eclipse.gef.Request;
import org.eclipse.gef.RequestConstants;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.editpolicies.DirectEditPolicy;
import org.eclipse.gef.requests.DirectEditRequest;
import org.eclipse.jface.viewers.CellEditor;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CommandWrap4EMF;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.PlanElementEditPart;

public class PMLDirectEditPolicy extends DirectEditPolicy {

	@Override
	protected Command getDirectEditCommand(DirectEditRequest request) {
		Command cmd = null;
		CellEditor editor = request.getCellEditor();
		if (editor == null) {
			return cmd;
		}
		
		// Only allow return the command if the label is non-empty.
		String labelText = (String) editor.getValue();
		if (labelText != null) {
			labelText = labelText.trim();
			if (labelText.length() > 0) {
				PlanElement elem = (PlanElement)getHost().getModel();
//				if(elem instanceof Plan)
//					cmd = new SetPlanNameCommand(elem, labelText);
//				else
//					cmd = new SetNameCommand(elem, labelText);
				TransactionalEditingDomain editingDomain = TransactionUtil.getEditingDomain(elem);
				return new CommandWrap4EMF(SetCommand.create(editingDomain, elem, AlicaPackage.eINSTANCE.getPlanElement_Name(), labelText));
			}
		}
		
		return cmd;
	}

	@Override
	protected void showCurrentEditValue(DirectEditRequest request) {
		System.err.println("REQUEST " + request);
		String value = (String) request.getCellEditor().getValue();
		System.out.println("VERSUCHT IWAS " + value);
		if (getHost() instanceof PlanElementEditPart) {
			PlanElementEditPart part = (PlanElementEditPart) getHost();
			part.getNameLabel().setText(value);
		}
	}
}