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
import java.util.Collections;
import java.util.List;

import org.eclipse.draw2d.ColorConstants;
import org.eclipse.draw2d.ConnectionAnchor;
import org.eclipse.draw2d.Ellipse;
import org.eclipse.draw2d.EllipseAnchor;
import org.eclipse.draw2d.IFigure;
import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.gef.ConnectionEditPart;
import org.eclipse.gef.DragTracker;
import org.eclipse.gef.Request;
import org.eclipse.gef.tools.DragEditPartsTracker;

import de.uni_kassel.vs.cn.planDesigner.alica.Edge;
import de.uni_kassel.vs.cn.planDesigner.alica.Node;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.ExecuteAndRefreshGraphCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.RolesetEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.RolesetEditorUtils;

public class NodeEditPart extends AbstractElementEditPart {
	
	@Override
	protected void addAllAdapters() {
		super.addAllAdapters();
		
		/// Register the editPart with the ui extensionmodel
		PmlUiExtension extension = CommonUtils.getUIAwareEditorAdapter(this).getUIExtension(getEObjectModel(), true);
		if(extension != null)
			this.adapter.addToObject(extension);
	}

	@Override
	protected IFigure createFigure() {
		Ellipse f = new Ellipse();
		f.setPreferredSize(15, 15);
		f.setBackgroundColor(ColorConstants.black);
		f.setOpaque(true);
		
		return f;
	}

	@Override
	protected void createEditPolicies() {
		// TODO Auto-generated method stub

	}
	
	@Override
	public DragTracker getDragTracker(Request request) {
		return new DragEditPartsTracker(this){
			@Override
			protected boolean handleDoubleClick(int button) {
				if(button == 1){
					setCollapsed(!isCollapsed());
				}
				System.out.println("Node (" +this +") is now " +(isCollapsed() ? "collapsed!" : "expanded"));
				return true;
			}
		};
	}
	
	public boolean isCollapsed() {
		// Try to get the ui extension 
		PmlUiExtension ext = CommonUtils.getUIAwareEditorAdapter(this).getUIExtension(getEObjectModel(), true);
		
		// If the ui extension is available return the collapsed feature, else fall back.
		return ext != null ? ext.isCollapsed() : true;
	}
	
	public void setCollapsed(boolean collapsed) {
		// Try to get the ui extension 
		PmlUiExtension ext = CommonUtils.getUIAwareEditorAdapter(this).getUIExtension(getEObjectModel(), true);
		
		if(ext != null){
			TaskGraph graph = (TaskGraph)getNode().eContainer();
			RolesetEditor editor = RolesetEditorUtils.getRolesetEditor(this);
			Command cmd = SetCommand.create(
					editor.getEditingDomain(), 
					ext, 
					PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Collapsed(), 
					collapsed);
			String label = collapsed ? "Collapse" : "Expand" +" Node" ;
			editor.getEMFCommandStack().execute(new ExecuteAndRefreshGraphCommand(label,cmd,getViewer(),graph));
		}
	}
	
	protected Node getNode(){
		return (Node)getModel();
	}
	
	@Override
	protected List getModelSourceConnections() {
		// We don't want to return connections if this node
		// is collapsed, since there is no node at the other end
		if(!isCollapsed()){
			return Collections.checkedList(getNode().getOutEdge(), Edge.class);
		}else
			return Collections.emptyList();
		
	}
	
	@Override
	protected List<Edge> getModelTargetConnections() {
		// Return only those edges which come from parents which 
		// are not collapsed
		List<Edge> edges = new ArrayList<Edge>();
		RolesetEditor editor = RolesetEditorUtils.getRolesetEditor(this);
		PmlUiExtension ext = null;
		for(Edge e : getNode().getInEdge()){
			Node parent = e.getFrom();
			ext = editor.getUIExtension(parent, false);
			if(ext == null || (ext != null && !ext.isCollapsed()))
				edges.add(e);
		}
		return edges;
	}

	@Override
	public ConnectionAnchor getSourceConnectionAnchor(
			ConnectionEditPart connection) {
		return new EllipseAnchor(getFigure());
	}

	@Override
	public ConnectionAnchor getSourceConnectionAnchor(Request request) {
		return new EllipseAnchor(getFigure());
	}

	@Override
	public ConnectionAnchor getTargetConnectionAnchor(
			ConnectionEditPart connection) {
		return new EllipseAnchor(getFigure());
	}

	@Override
	public ConnectionAnchor getTargetConnectionAnchor(Request request) {
		return new EllipseAnchor(getFigure());
	}

}
