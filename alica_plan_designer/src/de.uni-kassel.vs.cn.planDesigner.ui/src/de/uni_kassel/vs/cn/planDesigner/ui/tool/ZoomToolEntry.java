/*******************************************************************************
 * Copyright (c) 2005 IBM Corporation and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *     IBM Corporation - initial API and implementation
 *******************************************************************************/
package de.uni_kassel.vs.cn.planDesigner.ui.tool;

import org.eclipse.gef.Tool;
import org.eclipse.gef.palette.ToolEntry;
import org.eclipse.jface.resource.ImageDescriptor;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;


public class ZoomToolEntry extends ToolEntry {
	public ZoomToolEntry() {
		this("Zoom in/out"); 
	}

	public ZoomToolEntry(String label) {
		this(label, null);
	}

	public ZoomToolEntry(String label, String shortDesc) {
		super(
			label,
			shortDesc,
			ImageDescriptor.createFromImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_ZOOM_IN_OUT_16)), // these aren't the ones we want but leave them as placeholders
			ImageDescriptor.createFromImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_ZOOM_IN_OUT_24)));
		setUserModificationPermission(PERMISSION_NO_MODIFICATION);
	}

	@Override
	public Tool createTool() {
		return new ZoomTool();
	}
}
