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
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.eclipse.draw2d.FreeformLayer;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.MarginBorder;
import org.eclipse.draw2d.geometry.Rectangle;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.editparts.AbstractGraphicalEditPart;
import org.eclipse.swt.widgets.Control;
import org.eclipse.zest.layouts.InvalidLayoutConfiguration;
import org.eclipse.zest.layouts.LayoutAlgorithm;
import org.eclipse.zest.layouts.LayoutEntity;
import org.eclipse.zest.layouts.LayoutStyles;
import org.eclipse.zest.layouts.algorithms.CompositeLayoutAlgorithm;
import org.eclipse.zest.layouts.algorithms.HorizontalShift;
import org.eclipse.zest.layouts.algorithms.TreeLayoutAlgorithm;
import org.eclipse.zest.layouts.exampleStructures.SimpleNode;
import org.eclipse.zest.layouts.exampleStructures.SimpleRelationship;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Edge;
import de.uni_kassel.vs.cn.planDesigner.alica.Node;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskWrapper;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.ExecuteAndRefreshGraphCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.SetUIExtensionCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.RolesetEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.layout.DesignerFreeformLayout;
import de.uni_kassel.vs.cn.planDesigner.ui.layout.LayoutData;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.DesignerXYLayoutEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.util.RolesetEditorUtils;

public class TaskGraphEditPart extends AbstractGraphicalEditPart {
	
	private RolesetEditor editor;

	@Override
	protected IFigure createFigure() {
		IFigure layer = new FreeformLayer();
		layer.setLayoutManager(new DesignerFreeformLayout(this));
		layer.setBorder(new MarginBorder(5));
		return layer;
	}

	@Override
	protected void createEditPolicies() {
		installEditPolicy(EditPolicy.LAYOUT_ROLE, new DesignerXYLayoutEditPolicy(){
			// Disable resizing of nodes
			@Override
			protected EditPolicy createChildEditPolicy(EditPart child) {
//				return new NonResizableEditPolicy();
				return super.createChildEditPolicy(child);
			}
		});
	}
	
	public void expandOrCollapseAll(boolean collapse){
		CompoundCommand cmp = new CompoundCommand();
		RolesetEditor editor = RolesetEditorUtils.getRolesetEditor(this);
		PmlUiExtension ext = null;
		for(Node n : getTaskGraph().getNodes()){
			ext = editor.getUIExtension(n, true);
			
			cmp.append(SetCommand.create(
					editor.getEditingDomain(), 
					ext, 
					PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Collapsed(), 
					collapse));
		}
		
		getEditor().getEMFCommandStack().execute(
				new ExecuteAndRefreshGraphCommand(
						(collapse ? "Collapse" : "Expand") +" all",
								cmp,
								getViewer(),
								getTaskGraph()));
	}
	
	@Override
	protected void refreshVisuals() {
		// Here we call refresh on each child to handle
		// source and target connection refreshing. Maybe not the best
		// place to do it, but the easiest...
		List<AbstractGraphicalEditPart> children = getChildren();
		for(AbstractGraphicalEditPart child : children)
			child.refresh();
	}
	
	
	@Override
	protected List getModelChildren() {
		// Get the root node
		Node root = null;
		for(Node n : getTaskGraph().getNodes()){
			if(n.eClass() == AlicaPackage.eINSTANCE.getNode()){
				root = n;
				break;
			}
		}
		
		Set<Node> children = new HashSet<Node>();
		Set<Node> visited = new HashSet<Node>();
		if(root != null){
			children.add(root);
			visited.add(root);
			if(!getEditor().getUIExtension(root, true).isCollapsed())
				children.addAll(getVisibleChildNodes(root,visited));
		}else{
			// That should never be happen
		}
			
		return new ArrayList<Node>(children);
//		return getTaskGraph().getNodes();
	}
	
	private Set<Node> getVisibleChildNodes(Node node, Set<Node> visited){
		Set<Node> children = new HashSet<Node>();
		for(Edge e : node.getOutEdge()){
			Node child = e.getTo();
			if(!visited.contains(child)){
				// Add the child to the list
				children.add(child);
				visited.add(child);
				// If the child node is not collapsed,
				// also add the children
				if(!getEditor().getUIExtension(child, true).isCollapsed())
					children.addAll(getVisibleChildNodes(child,visited));
			}
			
		}
		return children;
	}
	
	public void doAutoLayout(){
		TreeLayoutAlgorithm algo = new TreeLayoutAlgorithm(LayoutStyles.NO_LAYOUT_NODE_RESIZING);
		HorizontalShift shift = new HorizontalShift(LayoutStyles.NO_LAYOUT_NODE_RESIZING);
		
		LayoutAlgorithm[] layouts = new LayoutAlgorithm[]{algo,shift};
		
		LayoutData data = getLayoutData();
		SimpleNode[] nodes = data.getNodes();
		
		SimpleRelationship[] edges = data.getEdges();
		Control c = getViewer().getControl();
		
		//Lexografisches sortieren von den Task im Roleset
		for(int i = 0; i < nodes.length ; i++){
			if(nodes[i].getRealObject() instanceof TaskWrapper){
				TaskWrapper firstTask = (TaskWrapper) nodes[i].getRealObject();
				for(int b = 0; b < nodes.length; b++){
					if(nodes[b].getRealObject() instanceof TaskWrapper){
						TaskWrapper secondTask = (TaskWrapper) nodes[b].getRealObject();
						if(firstTask.getTask().getName().compareToIgnoreCase(secondTask.getTask().getName()) < 0){
							SimpleNode newNode = nodes[b];
							nodes[b] = nodes[i];
							nodes[i] = newNode;
						}
					}
				}
			}
		}
		
		CompositeLayoutAlgorithm composite = new CompositeLayoutAlgorithm(LayoutStyles.NO_LAYOUT_NODE_RESIZING,layouts);
		try {
			composite.applyLayout(nodes, edges, 0, 0, c.getBounds().width, c.getBounds().height, false, false);
		} catch (InvalidLayoutConfiguration e) {
			e.printStackTrace();
		}
		
		RolesetEditor editor = RolesetEditorUtils.getRolesetEditor(this);
		
		CompoundCommand cmp = new CompoundCommand("Auto layout");
		for(SimpleNode ent : nodes){
//			System.out.println("Node: " +ent +" - (" +ent.getXInLayout() +"," +ent.getYInLayout() +"," +ent.getWidthInLayout() +"," +ent.getHeightInLayout() +")");
			EObject model = (EObject)ent.getRealObject();
			
			Rectangle constraint = new Rectangle(
					(int)ent.getXInLayout(),
					(int)ent.getYInLayout(),
					(int)ent.getWidthInLayout(),
					(int)ent.getHeightInLayout());
			
			// Ensure that we have an UIExtension
//			PmlUiExtension ext = editor.getUIExtension(model, true);
			cmp.append(new SetUIExtensionCommand(model, constraint, editor.getEditingDomain(), editor.getUIExtensionMap()));
		}
		editor.getEMFCommandStack().execute(cmp);
//		System.out.println("Zest layout finish");
	}
	
	/**
	 * Convenience method which casts the model to a TaskGraph
	 * @return
	 */
	private TaskGraph getTaskGraph(){
		return (TaskGraph)getModel();
	}

	public LayoutData getLayoutData() {
		LayoutData data = new LayoutData();
		
		TaskGraph graph = getTaskGraph();
		
		List<Node> graphNodes = getModelChildren();
		
		SimpleNode[] nodes = new SimpleNode[graphNodes.size()];
		
		Map<Node, LayoutEntity> mapping = new HashMap<Node, LayoutEntity>();
		for(int i=0; i < graphNodes.size(); i++){
			Node graphNode = graphNodes.get(i);
			IFigure groupFigure = ((AbstractGraphicalEditPart)getViewer().getEditPartRegistry().get(graphNode)).getFigure();
			SimpleNode node = new SimpleNode(graphNode,0,0,groupFigure.getPreferredSize().width, groupFigure.getPreferredSize().height);
			nodes[i] = node;
			mapping.put(graphNode, node);
		}
		
		List<Edge> graphEdges = new ArrayList<Edge>();
		for(Edge e : graph.getEdges()){
			// Add the edge only if both target and source are
			// in the list of graphNodes
			if(graphNodes.contains(e.getFrom()) && graphNodes.contains(e.getTo()))
				graphEdges.add(e);
		}
		
		SimpleRelationship[] edges = new SimpleRelationship[graphEdges.size()];
		for(int i=0; i < graphEdges.size(); i++){
			Edge edge = graphEdges.get(i);
			SimpleRelationship relation = new SimpleRelationship(mapping.get(edge.getFrom()), mapping.get(edge.getTo()), false);
			edges[i] = relation;
		}
		
		data.setEdges(edges);
		data.setNodes(nodes);
		
		return data;
			
	}
	public void updateThatPlan(){
		refreshVisuals();
		
	}

	protected RolesetEditor getEditor() {
		if(editor == null){
			editor = RolesetEditorUtils.getRolesetEditor(this);
		}
		return editor;
	}

}
