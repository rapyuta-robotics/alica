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

import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.Label;
import org.eclipse.draw2d.LineBorder;
import org.eclipse.draw2d.MarginBorder;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.editpolicies.SelectionEditPolicy;

public class ExitPointResultEditPart extends PlanElementEditPart {

	@Override
	protected IFigure createFigure() {
		return getNameLabel();
	}
	
	@Override
	protected Label createNameLabel() {
		return new Label(getPlanElement().getName());
	}

	@Override
	protected void createEditPolicies() {
		super.createEditPolicies();
		installEditPolicy(EditPolicy.LAYOUT_ROLE, new SelectionEditPolicy(){

			@Override
			protected void hideSelection() {
				if(getNameLabel() != null)
					getNameLabel().setBorder(new MarginBorder(1));
			}

			@Override
			protected void showSelection() {
				if(getNameLabel() != null)
					getNameLabel().setBorder(new LineBorder(1));
			}
			
		});
	}
	
	@Override
	protected void refreshVisuals() {
		((Label)getFigure()).setText(getPlanElement().getName());
	}

}
