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
import org.eclipse.draw2d.geometry.Rectangle;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.GraphicalEditPart;

import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.UIAwareEditor;

/**
 * A simple XY Layout that knows how to create missing constraints.
 * @author Zenobios
 *
 */
public class PMLXYLayout extends XYLayout {
	
	private EditPart owner;
	
	public PMLXYLayout(EditPart owner) {
		this.owner = owner;
	}
	
	@Override
	public Object getConstraint(IFigure figure) {
		Rectangle constraint = (Rectangle)super.getConstraint(figure);
		
		UIAwareEditor editor = CommonUtils.getUIAwareEditorAdapter(owner);
		EditPart editPart = PlanEditorUtils.mapFigure2EditPart(owner.getViewer(), figure);
		PmlUiExtension extension = editor.getUIExtension((EObject)editPart.getModel(), true);
		
		Rectangle userConstraint = new Rectangle(
				extension.getXPos(),
				extension.getYPos(),
				extension.getWidth(),
				extension.getHeight());
		
		if(constraint == null){
			
			constraint = userConstraint;
			
			((GraphicalEditPart)owner).setLayoutConstraint(editPart, figure, constraint);
		}
		
		Dimension preferredSize = figure.getPreferredSize();
		if(userConstraint.width < preferredSize.width)
		{
			constraint.width = preferredSize.width;
		}
		if(userConstraint.height < preferredSize.height)
		{
			constraint.height = preferredSize.height;
		}
		
		
		return constraint;
	}
}
