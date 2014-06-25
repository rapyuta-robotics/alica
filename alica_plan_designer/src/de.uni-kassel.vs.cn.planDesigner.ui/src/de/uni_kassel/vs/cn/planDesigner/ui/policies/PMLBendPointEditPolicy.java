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

import java.util.Collections;

import org.eclipse.draw2d.Connection;
import org.eclipse.draw2d.geometry.Point;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.RemoveCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.editpolicies.BendpointEditPolicy;
import org.eclipse.gef.requests.BendpointRequest;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.Bendpoint;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelFactory;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CommandWrap4EMF;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public class PMLBendPointEditPolicy extends BendpointEditPolicy {

	@Override
	protected Command getCreateBendpointCommand(BendpointRequest request) {
		// Get the location of the new bendpoint (absolute)
		Point loc = request.getLocation();
		
		// Translate the point relativ to the connection
		EditPart editPart = getHost();
		Connection conn = getConnection();
		conn.translateToRelative(loc);

		// Get the transition model
		EObject owner = (EObject)editPart.getModel();
		
		// Get the editor
		PlanEditor editor = PlanEditorUtils.getPlanEditor(editPart);
		
		// Get the UI extension
		PmlUiExtension ext = editor.getUIExtension(owner, true);
		
		// Get the editingDomain
		PMLTransactionalEditingDomain domain = editor.getEditingDomain();
		
		// Create a new Bendpoint
		Bendpoint bp = PmlUIExtensionModelFactory.eINSTANCE.createBendpoint();
		bp.setXPos(loc.x);
		bp.setYPos(loc.y);
		
//		org.eclipse.emf.common.command.Command cmd = domain.createCommand(
//				CreateChildCommand.class, 
//				new CommandParameter(
//						ext, 
//						PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Bendpoints(), 
//						new CommandParameter(null,null,bp),
//						request.getIndex()));
		org.eclipse.emf.common.command.Command cmd = new CreateChildCommand(
				domain, 
				ext, 
				PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Bendpoints(), 
				bp,
				request.getIndex(),
				Collections.emptyList());
		
		return new CommandWrap4EMF(cmd);
	}

	@Override
	protected Command getDeleteBendpointCommand(BendpointRequest request) {
		EditPart editPart = getHost();
		
		// Get the transition model
		EObject owner = (EObject)editPart.getModel();
		
		// Get the editor
		PlanEditor editor = PlanEditorUtils.getPlanEditor(editPart);
		
		// Get the UI extension
		PmlUiExtension ext = editor.getUIExtension(owner, true);
		
		// Get the editingDomain
		PMLTransactionalEditingDomain domain = editor.getEditingDomain();
		
		// Get the bendpoint to move
		Bendpoint bp = ext.getBendpoints().get(request.getIndex());
		
//		org.eclipse.emf.common.command.Command cmd = DeleteCommand.create(domain, bp);
		org.eclipse.emf.common.command.Command cmd = RemoveCommand.create(
				domain, 
				ext,
				PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Bendpoints(),
				bp); 
			
		return new CommandWrap4EMF(cmd);
	}

	@Override
	protected Command getMoveBendpointCommand(BendpointRequest request) {
		// Get the location of the moved bendpoint (absolute)
		Point loc = request.getLocation();
		
		// Translate the point relativ to the connection
		EditPart editPart = getHost();
		Connection conn = getConnection();
		conn.translateToRelative(loc);
		
		// Get the transition model
		EObject owner = (EObject)editPart.getModel();
		
		// Get the editor
		PlanEditor editor = PlanEditorUtils.getPlanEditor(editPart);
		
		// Get the UI extension
		PmlUiExtension ext = editor.getUIExtension(owner, true);
		
		// Get the editingDomain
		PMLTransactionalEditingDomain domain = editor.getEditingDomain();
		
		// Get the bendpoint to move
		Bendpoint bp = ext.getBendpoints().get(request.getIndex());
		
		CompoundCommand cmp = new CompoundCommand("Move bendpoint");
		
		cmp.append(SetCommand.create(domain, bp, PmlUIExtensionModelPackage.eINSTANCE.getBendpoint_XPos(), loc.x));
		cmp.append(SetCommand.create(domain, bp, PmlUIExtensionModelPackage.eINSTANCE.getBendpoint_YPos(), loc.y));
		
		return new CommandWrap4EMF(cmp);
	}

}
