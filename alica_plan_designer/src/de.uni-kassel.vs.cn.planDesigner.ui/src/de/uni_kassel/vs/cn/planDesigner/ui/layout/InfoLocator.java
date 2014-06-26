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
package de.uni_kassel.vs.cn.planDesigner.ui.layout;

import org.eclipse.draw2d.AbstractLocator;
import org.eclipse.draw2d.FigureCanvas;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.Viewport;
import org.eclipse.draw2d.geometry.Dimension;
import org.eclipse.draw2d.geometry.Point;
import org.eclipse.draw2d.geometry.Rectangle;
import org.eclipse.gef.GraphicalViewer;

public class InfoLocator extends AbstractLocator
{
	
	private final GraphicalViewer viewer;
	
	public InfoLocator(GraphicalViewer viewer)
	{
		this.viewer = viewer;
	}
	
	@Override
	public void relocate(IFigure target)
	{
		Dimension prefSize = target.getPreferredSize();
		Point location = getReferencePoint();
		target.translateToRelative(location);
		
		Rectangle rect = new Rectangle(location.x, location.y, getViewport().getBounds().width, prefSize.height);
		rect.translate(0, -target.getBounds().height);
		target.setBounds(rect);
	}

	@Override
	protected Point getReferencePoint()
	{
		Viewport viewport = getViewport();
		return viewport.getBounds().getBottomLeft();
	}

	private Viewport getViewport()
	{
		Viewport viewport = ((FigureCanvas)getViewer().getControl()).getViewport();
		return viewport;
	}

	private GraphicalViewer getViewer()
	{
		return viewer;
	}

	
	
}
