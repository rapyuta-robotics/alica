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

import java.util.List;
import java.util.Map;

import org.eclipse.draw2d.AbstractLayout;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.geometry.Dimension;
import org.eclipse.draw2d.geometry.Rectangle;
import org.eclipse.gef.editparts.AbstractGraphicalEditPart;
import org.eclipse.swt.widgets.Control;
import org.eclipse.zest.layouts.InvalidLayoutConfiguration;
import org.eclipse.zest.layouts.LayoutAlgorithm;
import org.eclipse.zest.layouts.LayoutStyles;
import org.eclipse.zest.layouts.algorithms.CompositeLayoutAlgorithm;
import org.eclipse.zest.layouts.algorithms.HorizontalShift;
import org.eclipse.zest.layouts.algorithms.TreeLayoutAlgorithm;
import org.eclipse.zest.layouts.exampleStructures.SimpleNode;
import org.eclipse.zest.layouts.exampleStructures.SimpleRelationship;

import de.uni_kassel.vs.cn.planDesigner.ui.parts.TaskGraphEditPart;

public class ZestTreeLayout extends AbstractLayout {
	
	private TaskGraphEditPart diagram;
	
	public ZestTreeLayout(TaskGraphEditPart diagram){
		this.diagram = diagram;
	}
	

	@Override
	protected Dimension calculatePreferredSize(IFigure container, int hint,
			int hint2) {
		container.validate();
		
		Rectangle bounds = new Rectangle();
		List<IFigure> children = container.getChildren();
		for(IFigure child : children){
			bounds.union(child.getBounds());
		}
		
		bounds.expand(container.getInsets());
		
		return bounds.getSize();
	}

	public void layout(IFigure container) {
		TreeLayoutAlgorithm algo = new TreeLayoutAlgorithm(LayoutStyles.NO_LAYOUT_NODE_RESIZING);
		HorizontalShift shift = new HorizontalShift(LayoutStyles.NO_LAYOUT_NODE_RESIZING);
		
		LayoutAlgorithm[] layouts = new LayoutAlgorithm[]{algo,shift};
		
		LayoutData data = diagram.getLayoutData();
		SimpleNode[] nodes = data.getNodes();
		SimpleRelationship[] edges = data.getEdges();
		Control c = diagram.getViewer().getControl();
		
		CompositeLayoutAlgorithm composite = new CompositeLayoutAlgorithm(LayoutStyles.NO_LAYOUT_NODE_RESIZING,layouts);
		try {
			composite.applyLayout(nodes, edges, 0, 0, c.getBounds().width, c.getBounds().height, false, false);
		} catch (InvalidLayoutConfiguration e) {
			e.printStackTrace();
		}
		
		Map<Object, AbstractGraphicalEditPart> registry = diagram.getViewer().getEditPartRegistry();
		
		for(SimpleNode ent : nodes){
			System.out.println("Node: " +ent +" - (" +ent.getXInLayout() +"," +ent.getYInLayout() +"," +ent.getWidthInLayout() +"," +ent.getHeightInLayout() +")");
			IFigure f = registry.get(ent.getRealObject()).getFigure();
			f.setBounds(new Rectangle(
					(int)ent.getXInLayout(),
					(int)ent.getYInLayout(),
					(int)ent.getWidthInLayout(),
					(int)ent.getHeightInLayout()));
			
		}
		
		
		System.out.println("Zest layout finish");
	}

}
