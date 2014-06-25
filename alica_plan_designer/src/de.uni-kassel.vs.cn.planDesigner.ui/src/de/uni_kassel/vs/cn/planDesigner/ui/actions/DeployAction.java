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
package de.uni_kassel.vs.cn.planDesigner.ui.actions;

import java.io.IOException;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.runtime.IPath;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.jface.action.IAction;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.ui.IObjectActionDelegate;
import org.eclipse.ui.IWorkbenchPart;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public class DeployAction implements IObjectActionDelegate {
	
	private ISelection selection;
	
	private IWorkbenchPart targetPart;

	public void setActivePart(IAction action, IWorkbenchPart targetPart) {
		this.targetPart = targetPart;
	}

	public void run(IAction action) {
		IFile file = (IFile)((StructuredSelection)selection).getFirstElement();
		
		ResourceSet rSet = CommonUtils.getAlicaResourceSet();
		Resource master = rSet.getResource(URI.createPlatformResourceURI(file.getFullPath().toString(),
				true), true);
		
		visit((Plan)master.getContents().get(0));
		
		// Ensure that all proxis are resolved
		EcoreUtil.resolveAll(rSet);
		
		StringBuffer buffer = new StringBuffer();
		buffer.append("Loaded Resources:\n");
		for(Resource r : rSet.getResources()){
			buffer.append("\t" + r.getURI() +"\n");
		}
		
		// Create the resource which will hold the merged resources
		IPath dir = file.getFullPath().removeLastSegments(1);
		dir = dir.append(PlanEditorUtils.removeFileExtension(file.getFullPath().lastSegment()) +".deploy");
		Resource merged = rSet.createResource(URI.createPlatformResourceURI(dir.toString(),
				true));
		
		// Add all resorces to the merged
		for(Resource r : rSet.getResources()){
			if(!r.equals(merged))
				merged.getContents().add(r.getContents().get(0));
		}
		
		try {
			merged.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		// Clean up
		rSet.getResources().clear();
		rSet = null;
		
		MessageDialog.openInformation(
				targetPart.getSite().getShell(), 
				"Deploy Plan", 
				buffer.toString());
	}
	
	private void visit(AbstractPlan p){
		if(p instanceof Plan)
			for(State s : ((Plan)p).getStates()){
				for(AbstractPlan plan : s.getPlans()){
					visit(plan);
				}
			}
	}

	public void selectionChanged(IAction action, ISelection selection) {
		this.selection = selection;
	}

}
