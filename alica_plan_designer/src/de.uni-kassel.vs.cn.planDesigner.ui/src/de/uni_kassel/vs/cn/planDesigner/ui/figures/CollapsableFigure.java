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
import org.eclipse.draw2d.FlowLayout;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.Label;
import org.eclipse.draw2d.MarginBorder;
import org.eclipse.draw2d.OrderedLayout;
import org.eclipse.draw2d.PositionConstants;
import org.eclipse.draw2d.geometry.Rectangle;
import org.eclipse.gef.handles.HandleBounds;
import org.eclipse.jface.resource.ImageRegistry;

import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public abstract class CollapsableFigure extends Figure implements HandleBounds{
	
	public static final int STYLE_LEFT_TO_RIGHT = 0;
	public static final int STYLE_TOP_TO_BOTTOM = 1;
	
	private int childrenStyle;
	
	private Label arrowLabel;
	
	private Figure childrenFigure;
	
	private IFigure descriptionFigure;
	
	private boolean collapsed;
	
	public CollapsableFigure() {
		this(STYLE_TOP_TO_BOTTOM);
	}
	
	public CollapsableFigure(int childrenStyle) {
		this.childrenStyle = childrenStyle;
		
		FlowLayout fl = new FlowLayout(false);
		fl.setStretchMinorAxis(true);
		setLayoutManager(fl);
		
		IFigure collapseChildWrap = new Figure();
		fl = new FlowLayout(false);
		fl.setMajorSpacing(0);
		fl.setMinorSpacing(0);
		collapseChildWrap.setLayoutManager(fl);
		
		IFigure collapseWrap = new Figure();
		fl = new FlowLayout();
		fl.setMinorAlignment(OrderedLayout.ALIGN_BOTTOMRIGHT);
		collapseWrap.setLayoutManager(fl);
		
		collapseWrap.add(getArrowLabel());
		collapseWrap.add(getDescriptionFigure());
		
		collapseChildWrap.add(collapseWrap);
		collapseChildWrap.add(getChildrenFigure());
		
		add(collapseChildWrap);
		
		hideCollapseControls();
	}
	
	public Label getArrowLabel() {
		if(arrowLabel == null){
			arrowLabel = new Label();
			arrowLabel.setLabelAlignment(PositionConstants.LEFT);
		}
		
		return arrowLabel;
	}
	
	public IFigure getDescriptionFigure(){
		if(descriptionFigure == null)
		{
			descriptionFigure = createDescriptionFigure();
		}
		
		return descriptionFigure;
	}
	
	/**
	 * Creates the descriptionFigure of this collapsableFigure. Subclasses should 
	 * override this method to return an appropriate figure.
	 * @return
	 */
	public abstract IFigure createDescriptionFigure();

	public Figure getChildrenFigure() {
		if(childrenFigure == null){
			childrenFigure = new Figure(); 
			FlowLayout fl = new FlowLayout();
			
			if(childrenStyle == STYLE_TOP_TO_BOTTOM)
				fl.setHorizontal(false);
			
			fl.setMajorSpacing(0);
			fl.setMinorSpacing(5);
			childrenFigure.setLayoutManager(fl);
			childrenFigure.setBorder(new MarginBorder(0,20,0,0));
			
		}
		return childrenFigure;
	}

	public boolean isCollapsed() {
		return collapsed;
	}

	public void setCollapsed(boolean collapsed) {
		this.collapsed = collapsed;
		
		ImageRegistry reg = PlanDesignerActivator.getDefault().getImageRegistry();
		
		// Set the correct icon
		getArrowLabel().setIcon(this.collapsed ? reg.get(
				PlanDesignerConstants.ICON_ARROW_RIGHT_10) : reg.get(PlanDesignerConstants.ICON_ARROW_DOWN_10));
	}
	
	public void hideCollapseControls(){
		getChildrenFigure().setVisible(false);
		
		ImageRegistry reg = PlanDesignerActivator.getDefault().getImageRegistry();
		// Set the correct icon
		
		getArrowLabel().setIcon(reg.get(PlanDesignerConstants.ICON_ARROW_RIGHT_DISABLE_10));
	}
	
	public void unhideCollapsControls(){
		ImageRegistry reg = PlanDesignerActivator.getDefault().getImageRegistry();
		
		// Set the correct icon
		getArrowLabel().setIcon(reg.get(PlanDesignerConstants.ICON_ARROW_RIGHT_10));
		getChildrenFigure().setVisible(true);
	}

	public void setChildrenStyle(int childrenStyle) {
		this.childrenStyle = childrenStyle;
		
		((FlowLayout)getChildrenFigure().getLayoutManager()).setHorizontal(childrenStyle == STYLE_LEFT_TO_RIGHT ? true : false);
	}
	
	
	public Rectangle getHandleBounds()
	{
		return getDescriptionFigure().getBounds();
	}

}
