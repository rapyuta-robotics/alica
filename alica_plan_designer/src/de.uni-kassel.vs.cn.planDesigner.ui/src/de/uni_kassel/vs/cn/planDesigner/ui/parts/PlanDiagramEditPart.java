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
import java.util.List;

import org.eclipse.draw2d.FreeformLayer;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.MarginBorder;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.editparts.AbstractGraphicalEditPart;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.ui.layout.DesignerFreeformLayout;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.DesignerXYLayoutEditPolicy;

public class PlanDiagramEditPart extends AbstractGraphicalEditPart {

	@Override
	protected IFigure createFigure() {
		IFigure layer = new FreeformLayer();
		layer.setLayoutManager(new DesignerFreeformLayout(this));
		layer.setBorder(new MarginBorder(5));
		return layer;
	}
	
	@Override
	protected void createEditPolicies() {
		installEditPolicy(EditPolicy.LAYOUT_ROLE, new DesignerXYLayoutEditPolicy());
	}
	
	@Override
	protected List<Plan> getModelChildren() {
		List<Plan> plan = new ArrayList<Plan>();
		for(EObject e : ((Resource)getModel()).getContents()){
			if(e instanceof Plan){
				plan.add((Plan)e);
			}
		}
		
		return plan;
	}

}
