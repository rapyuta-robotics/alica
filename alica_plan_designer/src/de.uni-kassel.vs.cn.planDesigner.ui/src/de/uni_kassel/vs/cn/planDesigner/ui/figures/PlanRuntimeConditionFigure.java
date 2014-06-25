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
package de.uni_kassel.vs.cn.planDesigner.ui.figures;

import org.eclipse.draw2d.Figure;
import org.eclipse.draw2d.Graphics;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.geometry.Rectangle;
import org.eclipse.swt.graphics.Color;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

/**
 * A figure which automatically sets it's visibility depending on 
 * the number of children it has.
 * @author Zenobios
 *
 */
public class PlanRuntimeConditionFigure extends Figure {
	
	private Color lineColor;
	
	private int borderWidth = 3;
	
	public PlanRuntimeConditionFigure() {
		super();
		setVisible(false);
	}
	
	@Override
	protected void paintFigure(Graphics graphics) {
		super.paintFigure(graphics);
		Rectangle b = getBounds();
		
		graphics.setBackgroundColor(getLineColor());
		
		graphics.fillRectangle(new Rectangle(b.x,b.y,b.width,getBorderWidth()));
	}
	
	private Color getLineColor(){
		if(lineColor == null){
			PlanDesignerActivator plugin = PlanDesignerActivator.getDefault();
			lineColor = plugin.getColorRegistry().get(PlanDesignerConstants.PLAN_LABEL_BACKGROUND_COLOR);
		}
		return lineColor;
	}
	
	@Override
	public void add(IFigure figure, Object constraint, int index) {
		super.add(figure, constraint, index);
		setVisible(true);
	}
	
	@Override
	public void remove(IFigure figure) {
		super.remove(figure);
		if(getChildren().size() == 0)
			setVisible(false);
	}

	public int getBorderWidth() {
		return borderWidth;
	}

	public void setBorderWidth(int borderWidth) {
		this.borderWidth = borderWidth;
	}

}
