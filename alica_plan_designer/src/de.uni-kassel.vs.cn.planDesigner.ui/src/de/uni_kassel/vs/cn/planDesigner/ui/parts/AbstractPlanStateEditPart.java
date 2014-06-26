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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.eclipse.draw2d.Figure;
import org.eclipse.draw2d.FlowLayout;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.MarginBorder;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.editpolicies.ComponentEditPolicy;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.ui.figures.CollapsableFigure;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PMLDirectEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PMLFlowLayoutEditPolicy;

public abstract class AbstractPlanStateEditPart extends CollapsableEditPart {
	
	@Override
	protected IFigure createFigure() {
		CollapsableFigure figure = (CollapsableFigure)super.createFigure();
		figure.setChildrenStyle(CollapsableFigure.STYLE_LEFT_TO_RIGHT);
		
		return figure;
	}
	
	@Override
	protected List<Object> getExpandedChildren() {
		List<Object> children = null;
		if(getAbstractPlan().getConditions().size() > 0){
			children = new ArrayList<Object>();
			children.addAll(getAbstractPlan().getConditions());
		}else
			children = Collections.emptyList();
		
		return children;
	}

	@Override
	protected void createEditPolicies() {
		// The Component EditPolicy is not needed here because we use the EMF delete
		// action to delete object. But we may forward the command creation to this
		// policy later if we decide that just using the action wasn't a good choice
		installEditPolicy(EditPolicy.COMPONENT_ROLE, new ComponentEditPolicy() { });
		installEditPolicy(EditPolicy.LAYOUT_ROLE, new PMLFlowLayoutEditPolicy());
		installEditPolicy(EditPolicy.DIRECT_EDIT_ROLE, new PMLDirectEditPolicy());
	}
	
	@Override
	protected IFigure createDescriptionFigure() {
		Figure rect = new Figure();
		rect.setBorder(new MarginBorder(1,3,1,3));
		FlowLayout fl = new FlowLayout(true);
		
		rect.setLayoutManager(fl);
		
		rect.add(getNameLabel());
		
		return rect;
	}
	
	/**
	 * Convenience method which returns the editParts model castet to an AbstractPlan
	 * @return
	 */
	private AbstractPlan getAbstractPlan(){
		return (AbstractPlan)getModel();
	}

}
