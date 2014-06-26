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

import org.eclipse.draw2d.AbstractConnectionAnchor;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.geometry.Point;
import org.eclipse.draw2d.geometry.PrecisionPoint;
import org.eclipse.draw2d.geometry.Rectangle;

public class SideConnectionAnchor extends AbstractConnectionAnchor {

	public static final int TOP = 0;
	public static final int BOTTOM = 1;
	public static final int LEFT = 2;
	public static final int RIGHT = 3;

	private int location;
	private int inset;
	// The offset is measured from the top (for LEFT and RIGHT) or the left (for TOP or BOTTOM)
	private int offset;
	
	public SideConnectionAnchor(IFigure owner, int location, int inset, int offset) {
		super(owner);
		this.location = location;
		this.inset = inset;
		this.offset = offset;
	}
	
	public Point getLocation(Point reference) {
		Rectangle r = getOwner().getBounds();
		int x, y;
		switch (location) {
			case TOP:
				x = r.x + offset;
				y = r.y + inset;
				break;
			case BOTTOM:
				x = r.x + offset;
				y = r.bottom() - inset;
				break;
			case LEFT:
				x = r.x + inset;
				y = r.y + offset;
				break;
			case RIGHT:
				x = r.right() - inset;
				y = r.y + offset;
				break;
			default:
				// Something went wrong. Attach the anchor to the middle
				x = r.right() - r.width / 2;
				y = r.bottom() - r.height / 2;
		}
		Point p = new PrecisionPoint(x,y);
		getOwner().translateToAbsolute(p);
		return p;
	}
	
	@Override
	public Point getReferencePoint() {
		return getLocation(null);
	}

}
