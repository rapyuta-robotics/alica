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



import org.eclipse.draw2d.ColorConstants;
import org.eclipse.draw2d.GridData;
import org.eclipse.draw2d.GridLayout;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.Label;
import org.eclipse.draw2d.MarginBorder;
import org.eclipse.draw2d.PositionConstants;
import org.eclipse.draw2d.geometry.Dimension;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.Request;
import org.eclipse.gef.RequestConstants;
import org.eclipse.gef.editpolicies.NonResizableEditPolicy;
import org.eclipse.gef.tools.DirectEditManager;
import org.eclipse.jface.resource.ColorRegistry;
import org.eclipse.jface.viewers.TextCellEditor;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Color;

import de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.figures.GradientFigure;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.TaskPriorityDirectEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PMLCellEditorLocator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.TaskPriorityDirectEditManager;

public class RoleTaskMappingEditPart extends AbstractElementEditPart {
	
	private Label roleNameLabel;
	
	private Label priorityLabel;
	
	/**
	 * The DirectEditManager is responsible for handling the in place editing of property values.
	 */
	private DirectEditManager directEditManager;
	
	private GradientFigure main;
	
	@Override
	protected IFigure createFigure() {
		main = new GradientFigure();
		main.setForegroundColor(computeColor());
		main.setBackgroundColor(ColorConstants.white);
		
		main.setBorder(new MarginBorder(2));
//		main.setBackgroundColor(ColorConstants.red);
		
		main.setLayoutManager(new GridLayout(2,false){
			@Override
			protected Dimension calculatePreferredSize(IFigure container,
					int hint, int hint2) {
				// We don't want the GridLayout to use any width/height from 
				// the parent so ignore the hints
				return super.calculatePreferredSize(container, SWT.DEFAULT, SWT.DEFAULT);
			}
		});
		GridData gData = new GridData(SWT.FILL,SWT.FILL,true,true);
		gData.widthHint = 70;
		gData.heightHint = 20;
		main.add(getRoleNameLabel(),gData);
		gData = new GridData(SWT.RIGHT,SWT.FILL,false,true);
		gData.heightHint = 20;
		main.add(getPriorityLabel(),gData);
		
		return main;
	}

	private Color computeColor() {
		double prio = getRoleTaskMapping().getPriority();
		ColorRegistry reg = PlanDesignerActivator.getDefault().getColorRegistry();
		Color result = null;
		
		if(prio == 0.5)
			result =  reg.get(PlanDesignerConstants.PRIORITY_DEFAULT_COLOR);		
		else if(prio < 0)
			result = reg.get(PlanDesignerConstants.PRIORITY_0_COLOR);
		else if(prio < 0.5)
			result = reg.get(PlanDesignerConstants.PRIORITY_1_COLOR);
		else 
			result = reg.get(PlanDesignerConstants.PRIORITY_2_COLOR);
		
		return result;
	}

	@Override
	protected void createEditPolicies() {
		installEditPolicy(EditPolicy.DIRECT_EDIT_ROLE, new TaskPriorityDirectEditPolicy());
		installEditPolicy(EditPolicy.SELECTION_FEEDBACK_ROLE, new NonResizableEditPolicy());
	}
	
	@Override
	protected void refreshVisuals() {
		super.refreshVisuals();
		// Refresh the priority label
		getPriorityLabel().setText(String.valueOf(getRoleTaskMapping().getPriority()));
		main.setForegroundColor(computeColor());
		main.setBackgroundColor(ColorConstants.white);
	}
	
	/**
	 * Invoke direct edit on the receiver
	 */
	public void performDirectEdit() {
		if (getPriorityLabel() != null) {
			getDirectEditManager().show();
		}
	}
	
	@Override
	public void performRequest(Request req) {
		if (req.getType() == RequestConstants.REQ_DIRECT_EDIT)
			// Only perfrom directEdit if we also have a corresponding
			// EditPolicy installed. Otherwise that really doesn't make sense
			if(getEditPolicy(EditPolicy.DIRECT_EDIT_ROLE) != null)
				performDirectEdit();
		else 
			super.performRequest(req);
		
	}
	
	protected DirectEditManager getDirectEditManager() {
		if(directEditManager == null){
			directEditManager = new TaskPriorityDirectEditManager(this,TextCellEditor.class, new PMLCellEditorLocator(getPriorityLabel()));
		}
			
		return directEditManager;
	}
	
	private InternalRoleTaskMapping getRoleTaskMapping(){
		return (InternalRoleTaskMapping)getModel();
	}

	public Label getRoleNameLabel() {
		if(roleNameLabel == null){
			roleNameLabel = new Label(getRoleTaskMapping().getRole().getName());
			roleNameLabel.setLabelAlignment(PositionConstants.LEFT);
//			roleNameLabel.setOpaque(true);
			roleNameLabel.setForegroundColor(ColorConstants.black);
		}
		return roleNameLabel;
	}

	public Label getPriorityLabel() {
		if(priorityLabel == null){
			priorityLabel = new Label(String.valueOf(getRoleTaskMapping().getPriority()));
			priorityLabel.setLabelAlignment(PositionConstants.RIGHT);
			priorityLabel.setForegroundColor(ColorConstants.black);
			
		}
		return priorityLabel;
	}

}
