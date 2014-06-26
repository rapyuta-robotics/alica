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

import org.eclipse.draw2d.Label;
import org.eclipse.gef.DragTracker;
import org.eclipse.gef.Request;
import org.eclipse.gef.RequestConstants;
import org.eclipse.gef.tools.DragEditPartsTracker;
import org.eclipse.jface.wizard.WizardDialog;

import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLPlanTypeConfigurationWizard;

public class PlanTypeEditPart extends AbstractPlanStateEditPart{
	
	@Override
	protected Label createNameLabel() {
		Label l = new Label(getPlanElement().getName());;
		
		PlanDesignerActivator plugin = PlanDesignerActivator.getDefault();
		
		l.setIcon(plugin.getImageRegistry().get(PlanDesignerConstants.ICON_PLANTYPE_16));
		
		return l;
	}
	
	@Override
	public DragTracker getDragTracker(Request request)
	{
		return new DragEditPartsTracker(this);
	}

	@Override
	public void performRequest(Request req)
	{
		if(req.getType() == RequestConstants.REQ_OPEN)
		{
			// Create a plantype configuration wizard and initialize it with the plantype
			PMLPlanTypeConfigurationWizard wiz = new PMLPlanTypeConfigurationWizard((PlanType)getModel());
			
			WizardDialog dialog = new WizardDialog(getViewer().getControl().getShell(),wiz);
			
			
			dialog.setBlockOnOpen(true);
			dialog.open();
		}
		else
		{
			super.performRequest(req);
		}
	}
}
