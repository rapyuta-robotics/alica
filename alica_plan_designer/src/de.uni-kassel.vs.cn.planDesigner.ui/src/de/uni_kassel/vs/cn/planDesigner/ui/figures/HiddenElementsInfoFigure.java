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


import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;

import org.eclipse.draw2d.ColorConstants;
import org.eclipse.draw2d.FigureCanvas;
import org.eclipse.draw2d.FlowLayout;
import org.eclipse.draw2d.Graphics;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.Label;
import org.eclipse.draw2d.Locator;
import org.eclipse.draw2d.MarginBorder;
import org.eclipse.draw2d.RectangleFigure;
import org.eclipse.gef.GraphicalViewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.Font;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class HiddenElementsInfoFigure extends RectangleFigure
{
	
	private class ViewportTracker implements PropertyChangeListener
	{
		public void propertyChange(PropertyChangeEvent evt)
		{
			relocate();
		}
	}
	
	private final Locator locator;
	private Label textFigure;
	private final GraphicalViewer viewer;
	
	private ViewportTracker viewportTracker;

	public HiddenElementsInfoFigure(GraphicalViewer viewer, Locator locator)
	{
		this.viewer = viewer;
		this.locator = locator;
		setLayoutManager(new FlowLayout());
		setBorder(new MarginBorder(3));
		add(getTextFigure());
		setOpaque(false);
	}
	
	@Override
	public void paint(Graphics graphics)
	{

		super.paint(graphics);
		
	}
	
	@Override
	protected void paintClientArea(Graphics graphics)
	{
		graphics.pushState();
		graphics.setAlpha(100);
		Color oldForeground = graphics.getForegroundColor();
		Color oldBackground = graphics.getBackgroundColor();
		graphics.setForegroundColor(ColorConstants.white);
		graphics.setBackgroundColor(ColorConstants.yellow);
		graphics.fillGradient(getBounds(), true);
		graphics.setForegroundColor(oldForeground);
		graphics.setBackgroundColor(oldBackground);
		graphics.popState();
		super.paintClientArea(graphics);
	}

	private void hookViewport()
	{
		((FigureCanvas)this.viewer.getControl()).getViewport().getHorizontalRangeModel().addPropertyChangeListener(getViewportTracker());
		((FigureCanvas)this.viewer.getControl()).getViewport().getVerticalRangeModel().addPropertyChangeListener(getViewportTracker());
	}
	
	private void unhookViewport()
	{
		((FigureCanvas)this.viewer.getControl()).getViewport().getHorizontalRangeModel().removePropertyChangeListener(getViewportTracker());
		((FigureCanvas)this.viewer.getControl()).getViewport().getVerticalRangeModel().removePropertyChangeListener(getViewportTracker());
	}
	
	private IFigure getTextFigure()
	{
		if(textFigure == null)
		{
			textFigure = new Label();
			textFigure.setFont(new Font(null,"",11,SWT.NORMAL));
			textFigure.setIcon(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_HIDDEN_16));
			textFigure.setText("Info: This plan contains hidden elements! Go to the hidden elements view to unhide these elements.");
		}
		
		return textFigure;
	}

	public void relocate()
	{
		locator.relocate(this);
	}

	@Override
	public void addNotify()
	{
		super.addNotify();
		hookViewport();
	}
	
	@Override
	public void removeNotify()
	{
		unhookViewport();
		super.removeNotify();
	}

	private ViewportTracker getViewportTracker()
	{
		if(viewportTracker == null)
		{
			viewportTracker = new ViewportTracker();
		}
		return viewportTracker;
	}
}
