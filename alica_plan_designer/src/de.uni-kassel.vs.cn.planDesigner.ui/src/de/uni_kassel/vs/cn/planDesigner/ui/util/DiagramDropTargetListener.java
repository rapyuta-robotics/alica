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
package de.uni_kassel.vs.cn.planDesigner.ui.util;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.Request;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.dnd.AbstractTransferDropTargetListener;
import org.eclipse.gef.dnd.TemplateTransfer;
import org.eclipse.gef.requests.CreateRequest;
import org.eclipse.gef.requests.CreationFactory;
import org.eclipse.swt.dnd.DND;

import de.uni_kassel.vs.cn.planDesigner.ui.policies.RequestConstants;

public class DiagramDropTargetListener extends AbstractTransferDropTargetListener {
	
	
	public DiagramDropTargetListener(EditPartViewer viewer) {
		super(viewer, TemplateTransfer.getInstance());
	}

	/**
	 * Assumes that the target request is a {@link CreateRequest}. 
	 */
	protected void updateTargetRequest() {
		CreateRequest request = getCreateRequest();
		request.setLocation(getDropLocation());
	}
	
	protected Request createTargetRequest() {
		// We create a special CreateRequest with type REQ_DROP to
		// handle the request in a seperate EditPolicy
		// (DragDropEditPolicy)
		return new CreateRequest(RequestConstants.REQ_DRAG);
	}
	
	@Override
	protected void handleDrop() {
		updateTargetEditPart();
		
		if(getTargetEditPart() != null)
			getCreateRequest().setFactory(getFactory());
		
		updateTargetRequest();
		
		// Set the type of the request to drop
		getCreateRequest().setType(RequestConstants.REQ_DROP);
		
		if (getTargetEditPart() != null) {
			Command command = getCommand();
			if (command != null && command.canExecute())
				getViewer().getEditDomain().getCommandStack().execute(command);
			else
				getCurrentEvent().detail = DND.DROP_NONE;
		} else
			getCurrentEvent().detail = DND.DROP_NONE;
	}
	
	private EObject getEObjectFromTransfer(){
		Object object = ((TemplateTransfer)getTransfer()).getTemplate();
		return object instanceof EObject ? (EObject)object : null;
	}
	
	/**
	 * A helper method that casts the target Request to a CreateRequest.
	 * @return CreateRequest
	 */
	protected final CreateRequest getCreateRequest() {
		return ((CreateRequest)getTargetRequest());
	}
	
	private CreationFactory getFactory(){
		if(getEObjectFromTransfer() != null){
			return new PassThroughCreationFactory(getEObjectFromTransfer());
		}else
			return null;
		
	}
	
}
