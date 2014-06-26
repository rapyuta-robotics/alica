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
import org.eclipse.jface.resource.ImageRegistry;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;

public class ConditionEditPart extends PlanElementEditPart {
	
	private String iconPath;
	
	public ConditionEditPart(String iconPath) {
		super();
		this.iconPath = iconPath;
	}

	@Override
	protected IFigure createFigure() {
		Label l = new Label();
		
		ImageRegistry reg = PlanDesignerActivator.getDefault().getImageRegistry();
		l.setIcon(reg.get(getIconPath()));
		
		return l;
	}

	private String getIconPath() {
		return iconPath;
	}


}
