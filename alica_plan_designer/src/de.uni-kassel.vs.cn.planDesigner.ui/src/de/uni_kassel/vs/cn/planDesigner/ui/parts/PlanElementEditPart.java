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
package de.uni_kassel.vs.cn.planDesigner.ui.parts;

import org.eclipse.draw2d.ConnectionAnchor;
import org.eclipse.draw2d.EllipseAnchor;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.KeyEvent;
import org.eclipse.draw2d.KeyListener;
import org.eclipse.draw2d.Label;
import org.eclipse.gef.ConnectionEditPart;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.Request;
import org.eclipse.gef.RequestConstants;
import org.eclipse.gef.tools.DirectEditManager;
import org.eclipse.jface.viewers.TextCellEditor;

import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PMLDirectEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PMLCellEditorLocator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PMLDirectEditManager;

public abstract class PlanElementEditPart extends AbstractElementEditPart{
	
	/**
	 * Every PlanElement can have a nameLabel. If it is set, it will
	 * automatically be updated during UI refresh.
	 */
	protected Label nameLabel;
	
	/**
	 * The DirectEditManager is responsible for handling the in place editing of property values.
	 */
	private DirectEditManager directEditManager;
	
	
	
	@Override
	protected void createEditPolicies() {
		installEditPolicy(EditPolicy.DIRECT_EDIT_ROLE, new PMLDirectEditPolicy());
		
	}
	
	/**
	 * Invoke direct edit on the receiver
	 */
	public void performDirectEdit() {
		if (getNameLabel() != null) {
			getDirectEditManager().show();
		}
	}
	
	
	
	@Override
	public void performRequest(Request req) {
		System.out.println("ICH REQUESTE : " + req);
		if (req.getType() == RequestConstants.REQ_DIRECT_EDIT)
			// Only perfrom directEdit if we also have a corresponding
			// EditPolicy installed. Otherwise that really doesn't make sense
			if(getEditPolicy(EditPolicy.DIRECT_EDIT_ROLE) != null){
				performDirectEdit();
			}
		else 
			super.performRequest(req);
		
	}
	
	public PlanElement getPlanElement(){
		return (PlanElement)getModel();
	}

	public Label getNameLabel() {
		if(this.nameLabel == null){
			this.nameLabel = createNameLabel();
		}
		return nameLabel;
	}
	
	protected Label createNameLabel(){
		return null;
	}
	
	@Override
	protected void refreshVisuals() {
		super.refreshVisuals();
		
		// Update the name label
		if(getNameLabel() != null){
			getNameLabel().setText(getPlanElement().getName());
		}
	}
	
	public IFigure getMainFigure(){
		return getFigure();
	}

	@Override
	public ConnectionAnchor getSourceConnectionAnchor(
			ConnectionEditPart connection) {
		return new EllipseAnchor(getMainFigure());
	}

	@Override
	public ConnectionAnchor getSourceConnectionAnchor(Request request) {
		return new EllipseAnchor(getMainFigure());
	}

	@Override
	public ConnectionAnchor getTargetConnectionAnchor(
			ConnectionEditPart connection) {
		EllipseAnchor ea = new EllipseAnchor(getMainFigure());
		return ea;
	}

	@Override
	public ConnectionAnchor getTargetConnectionAnchor(Request request) {
		return new EllipseAnchor(getMainFigure());
	}
	
	protected DirectEditManager getDirectEditManager() {
		if(directEditManager == null){
			directEditManager = new PMLDirectEditManager(this,TextCellEditor.class, new PMLCellEditorLocator(getNameLabel()));
		}
			
		return directEditManager;
	}
	
	
}

