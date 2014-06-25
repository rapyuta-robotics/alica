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

import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.XYLayout;
import org.eclipse.draw2d.geometry.Dimension;
import org.eclipse.draw2d.geometry.Point;
import org.eclipse.gef.EditPart;

/**
 * A PlanXYLayout which has freeform capabilities.
 * @author Zenobios
 *
 */
public class DesignerFreeformLayout extends PMLXYLayout {
	
	public DesignerFreeformLayout(EditPart owner) {
		super(owner);
	}
	
	/**
	 * Returns the point (0,0) as the origin.
	 * @see XYLayout#getOrigin(IFigure)
	 */
	public Point getOrigin(IFigure figure) {
		return new Point();
	}

	@Override
	public Dimension getPreferredSize(IFigure container, int hint, int hint2)
	{
		// TODO Auto-generated method stub
		return super.getPreferredSize(container, hint, hint2);
	}
}
