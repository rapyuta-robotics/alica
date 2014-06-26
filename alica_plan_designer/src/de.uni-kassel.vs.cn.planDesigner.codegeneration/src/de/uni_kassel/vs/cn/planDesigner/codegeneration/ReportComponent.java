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
package de.uni_kassel.vs.cn.planDesigner.codegeneration;

import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.emf.mwe.core.WorkflowContext;
import org.eclipse.emf.mwe.core.issues.Issues;
import org.eclipse.emf.mwe.core.lib.AbstractWorkflowComponent2;
import org.eclipse.emf.mwe.core.monitor.ProgressMonitor;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.swt.widgets.Display;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class ReportComponent extends AbstractWorkflowComponent2 {

	@Override
	public void checkConfigurationInternal(Issues arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void invokeInternal(WorkflowContext arg0, ProgressMonitor arg1,
			Issues arg2) {
		if (arg2.hasErrors()) {
			MessageDialog.openError(Display.getCurrent().getActiveShell(),
					"Code Generation Error",
					"There were errors during code generation!");
		} else if (arg2.getWarnings().length > 0) {
			MessageDialog.openWarning(Display.getCurrent().getActiveShell(),
					"Code Generation Warning",
					"There were warnings during code generation!");
		} else {
			MessageDialog.openInformation(
					Display.getCurrent().getActiveShell(),
					"Code Generation Complete", "Code generation successfull!");
		}

		String base = PlanDesignerActivator.getDefault().getPreferenceStore()
				.getString(PlanDesignerConstants.PREF_CODEGEN_BASE_PATH);

		IResource basePath = ResourcesPlugin.getWorkspace().getRoot()
				.findMember(base);
		
		try {
			basePath.refreshLocal(IResource.DEPTH_ONE, null);
		} catch (CoreException e) {
			e.printStackTrace();
		}
	}
}
