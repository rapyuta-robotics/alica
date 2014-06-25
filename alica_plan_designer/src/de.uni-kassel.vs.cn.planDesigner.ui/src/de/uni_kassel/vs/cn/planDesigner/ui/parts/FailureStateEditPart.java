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

import org.eclipse.draw2d.Figure;
import org.eclipse.draw2d.FlowLayout;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.ImageFigure;
import org.eclipse.jface.resource.ImageRegistry;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class FailureStateEditPart extends TerminalStateEditPart {

	@Override
	public IFigure getMainFigure() {
		if(pointFigure == null){
			ImageRegistry reg = PlanDesignerActivator.getDefault().getImageRegistry();
			pointFigure = new ImageFigure(reg.get(PlanDesignerConstants.ICON_FAILURE_POINT_15));	
		}
		return pointFigure;
	}
	@Override
	protected IFigure createDescriptionFigure() {
		IFigure wrapper = new Figure();
		FlowLayout fl = new FlowLayout(false);
		fl.setMajorSpacing(0);
		fl.setMinorSpacing(0);
		fl.setMinorAlignment(FlowLayout.ALIGN_CENTER);
		wrapper.setLayoutManager(fl);
		
		wrapper.add(getNameLabel());
		wrapper.add(getMainFigure());
		
		return wrapper;
	}

}
