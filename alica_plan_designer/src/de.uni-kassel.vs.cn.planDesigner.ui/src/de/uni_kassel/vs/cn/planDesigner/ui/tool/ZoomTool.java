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

import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.GraphicalViewer;
import org.eclipse.gef.editparts.ScalableFreeformRootEditPart;
import org.eclipse.gef.tools.AbstractTool;
import org.eclipse.swt.graphics.Cursor;
import org.eclipse.swt.graphics.ImageData;
import org.eclipse.swt.widgets.Display;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;



public class ZoomTool extends AbstractTool {
	
	protected Cursor cursor;
	
	public ZoomTool() {
		setDefaultCursor(createCursor());
	}
	
	@Override
	public void deactivate() {
		if (!isActive()) return;
		super.deactivate();
		if (cursor != null) {
			cursor.dispose();
			cursor = null;
		}
	}
	
	protected Cursor createCursor() {
		Display display = Display.getDefault();
		ImageData source = PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_CURSOR_ZOOM_IN_OUT).getImageData();
		ImageData mask = PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_CURSOR_ZOOM_MASK).getImageData();
		this.cursor = new Cursor(display, source, mask, 0, 0);
		return cursor;
	}
	
	@Override
	protected boolean handleButtonDown(int button) {
		if (button == 1) {
			EditPartViewer v = getCurrentViewer();
			if (v != null && v instanceof GraphicalViewer) {
				ScalableFreeformRootEditPart root = (ScalableFreeformRootEditPart)(v.getRootEditPart());
				root.getZoomManager().zoomIn();
			}
		}
		if (button == 3) {
			EditPartViewer v = getCurrentViewer();
			if (v != null && v instanceof GraphicalViewer) {
				ScalableFreeformRootEditPart root = (ScalableFreeformRootEditPart)(v.getRootEditPart());
				root.getZoomManager().zoomOut();
			}
		}
		return true;
	}

	@Override
	protected String getCommandName() {
		return null;
	}

	@Override
	protected String getDebugName() {
		return null;
	}
}
