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

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.edit.domain.EditingDomain;
import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.GraphicalEditPart;
import org.eclipse.gef.requests.DirectEditRequest;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;

public class SetNameAndDirectEditCommand extends SetCommand {

	private EditPartViewer viewer;
	
	public SetNameAndDirectEditCommand(EditingDomain domain, EObject owner, Object value, EditPartViewer viewer) {
		super(domain, owner, AlicaPackage.eINSTANCE.getPlanElement_Name(), value);
		this.viewer = viewer;
	}
	
	@Override
	public void doExecute() {
		super.doExecute();
		
		GraphicalEditPart editPart = (GraphicalEditPart)viewer.getEditPartRegistry().get(getOwner());
		if (editPart != null) {
			editPart.getFigure().getParent().getLayoutManager().invalidate();
			editPart.performRequest(new DirectEditRequest());
		}
	}

}
