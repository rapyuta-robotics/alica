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
package de.uni_kassel.vs.cn.planDesigner.ui;

import org.eclipse.gef.EditPart;
import org.eclipse.gef.EditPartFactory;

import de.uni_kassel.vs.cn.planDesigner.alica.Edge;
import de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.alica.Node;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskWrapper;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.EdgeEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.NodeEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.RoleTaskMappingEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.TaskGraphEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.TaskWrapperEditPart;

public class RolesetEditPartFactory implements EditPartFactory {

	public EditPart createEditPart(EditPart context, Object model) {
		EditPart child = null;
		
		if(model instanceof TaskGraph)
			child = new TaskGraphEditPart();
		else if(model instanceof TaskWrapper)
			child = new TaskWrapperEditPart();
		else if(model instanceof Node)
			child = new NodeEditPart();
		else if(model instanceof InternalRoleTaskMapping)
			child = new RoleTaskMappingEditPart();
		else if(model instanceof Edge)
			child = new EdgeEditPart();
		
		
		child.setModel(model);
		
		return child;
	}

}
