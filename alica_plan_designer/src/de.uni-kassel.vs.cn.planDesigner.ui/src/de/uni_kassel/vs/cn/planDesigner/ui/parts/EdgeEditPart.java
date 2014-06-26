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


import java.util.Map;

import org.eclipse.draw2d.Graphics;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.PolygonDecoration;
import org.eclipse.draw2d.PolylineConnection;
import org.eclipse.draw2d.ShortestPathConnectionRouter;
import org.eclipse.gef.editparts.AbstractConnectionEditPart;
import org.eclipse.gef.editparts.AbstractGraphicalEditPart;
import org.eclipse.swt.SWT;

import de.uni_kassel.vs.cn.planDesigner.alica.Edge;

public class EdgeEditPart extends AbstractConnectionEditPart {
	
	@Override
	protected IFigure createFigure() {
		PolylineConnection con = new PolylineConnection(){
			@Override
			public void paint(Graphics graphics) {
				graphics.setAntialias(SWT.ON);
				super.paint(graphics);
			}
		};
		PolygonDecoration arrow = new PolygonDecoration();
		arrow.setTemplate(PolygonDecoration.TRIANGLE_TIP);
		con.setTargetDecoration(arrow);
		
		// Try to get the figure where all elements are contained
		IFigure graphFigure = getTaskGraphFigure();
		if(graphFigure != null)
			con.setConnectionRouter(new ShortestPathConnectionRouter(getTaskGraphFigure()));
		else
			System.err.println("Fallback!!");
		
		return con;
	}

	private IFigure getTaskGraphFigure() {
		Edge e = getEdge();
		Map<Object, AbstractGraphicalEditPart> registry = getViewer().getEditPartRegistry();
		AbstractGraphicalEditPart part = registry.get(e.getTo());
		if(part == null)
			part = registry.get(e.getFrom());
		
		if(part != null)
			return part.getFigure();
		else
			return null;
	}
	
	private Edge getEdge(){
		return (Edge)getModel();
	}

	@Override
	protected void createEditPolicies() {
	}

}
