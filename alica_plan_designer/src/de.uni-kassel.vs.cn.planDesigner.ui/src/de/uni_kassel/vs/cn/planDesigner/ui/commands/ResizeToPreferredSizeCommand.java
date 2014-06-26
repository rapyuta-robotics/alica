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
package de.uni_kassel.vs.cn.planDesigner.ui.commands;

import java.util.Map;

import org.eclipse.draw2d.geometry.Dimension;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.gef.editparts.AbstractGraphicalEditPart;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Node;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.RolesetEditor;

public class ResizeToPreferredSizeCommand extends RecordingCommand {

	private RolesetEditor editor;

	public ResizeToPreferredSizeCommand(RolesetEditor editor) {
		super(editor.getEditingDomain(), "Resize elements");
		this.editor = editor;
	}

	@Override
	protected void doExecute() {
		// Get the Taskgraph
		TaskGraph graph = editor.getTaskGraph();
		// Get the registry
		Map<Object, AbstractGraphicalEditPart> registry = editor.getGraphicalViewer().getEditPartRegistry();
		PmlUiExtension ext = null;
		
		for(Node n : graph.getNodes()){
			if(n.eClass() == AlicaPackage.eINSTANCE.getTaskWrapper()){
				// Get the editpart which belongs to that model object
				AbstractGraphicalEditPart part = registry.get(n);
				
				// The editpart can be null in case there is no editpart
				// which belongs to this model node. This is the case
				// if the node is not visible, i.e. if it is collapsed
				if(part != null){
					// Get the preferred size of the figure
					Dimension prefSize = part.getFigure().getPreferredSize();
					// Get the UI Extension
					ext = editor.getUIExtension(n, true);
					
					// Set the new size
					ext.setWidth(prefSize.width);
					ext.setHeight(prefSize.height);
				}
			}
		}

	}

}
