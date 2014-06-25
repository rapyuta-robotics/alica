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


import java.util.ArrayList;
import java.util.List;

import org.eclipse.draw2d.ConnectionAnchor;
import org.eclipse.draw2d.Figure;
import org.eclipse.draw2d.FlowLayout;
import org.eclipse.draw2d.GroupBoxBorder;
import org.eclipse.draw2d.IFigure;
import org.eclipse.gef.ConnectionEditPart;
import org.eclipse.gef.Request;

import de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskWrapper;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.layout.SideConnectionAnchor;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.UIAwareEditor;


public class TaskWrapperEditPart extends NodeEditPart {
	
	private IFigure mainFigure;

	@Override
	protected IFigure createFigure() {
		// Create the containerFigure into which all children
		// will be inserted
		mainFigure = new Figure();
		FlowLayout fLayout = new FlowLayout(false);
		fLayout.setStretchMinorAxis(true);
		mainFigure.setLayoutManager(fLayout);
		
		
		GroupBoxBorder groupBorder = new GroupBoxBorder(getTaskWrapper().getTask().getName());
		mainFigure.setBorder(groupBorder);
		
		return mainFigure;
	}

	@Override
	protected List getModelChildren() {
		List<InternalRoleTaskMapping> children = new ArrayList<InternalRoleTaskMapping>();
		UIAwareEditor editor = CommonUtils.getUIAwareEditorAdapter(this);
		PmlUiExtension ext = null;
		for(InternalRoleTaskMapping mapping : getTaskWrapper().getMappings()){
			ext = editor.getUIExtension(mapping.getRole(), true);
			if(!ext.isCollapsed())
				children.add(mapping);
		}
		return children;
	}

	@Override
	protected void createEditPolicies() {
	
	}
	
	private TaskWrapper getTaskWrapper(){
		return (TaskWrapper)getModel();
	}

	@Override
	public ConnectionAnchor getSourceConnectionAnchor(
			ConnectionEditPart connection) {
		return new SideConnectionAnchor(getFigure(),SideConnectionAnchor.BOTTOM,7,getFigure().getPreferredSize().width/2);
	}

	@Override
	public ConnectionAnchor getSourceConnectionAnchor(Request request) {
		return new SideConnectionAnchor(getFigure(),SideConnectionAnchor.BOTTOM,7,getFigure().getPreferredSize().width/2);
	}

	@Override
	public ConnectionAnchor getTargetConnectionAnchor(
			ConnectionEditPart connection) {
		return new SideConnectionAnchor(getFigure(),SideConnectionAnchor.TOP,0,getFigure().getPreferredSize().width/2);
	}

	@Override
	public ConnectionAnchor getTargetConnectionAnchor(Request request) {
		return new SideConnectionAnchor(getFigure(),SideConnectionAnchor.TOP,0,getFigure().getPreferredSize().width/2);
	}

}
