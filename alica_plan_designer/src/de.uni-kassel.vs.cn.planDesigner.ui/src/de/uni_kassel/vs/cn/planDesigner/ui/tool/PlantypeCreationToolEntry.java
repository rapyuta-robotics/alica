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
package de.uni_kassel.vs.cn.planDesigner.ui.tool;

import org.eclipse.gef.Tool;
import org.eclipse.gef.palette.CombinedTemplateCreationEntry;
import org.eclipse.gef.requests.CreationFactory;
import org.eclipse.gef.tools.CreationTool;
import org.eclipse.jface.resource.ImageDescriptor;

public class PlantypeCreationToolEntry extends CombinedTemplateCreationEntry {
	
	public PlantypeCreationToolEntry(String label, String shortDesc,
			CreationFactory factory, ImageDescriptor iconSmall, ImageDescriptor iconLarge) {
		super(label,shortDesc,factory,iconSmall,iconLarge);
	}
	
	@Override
	public Tool createTool() {
		CreationTool tool = new PlanTypeCreationTool();
		tool.setProperties(getToolProperties());
		return tool;
	}

}
