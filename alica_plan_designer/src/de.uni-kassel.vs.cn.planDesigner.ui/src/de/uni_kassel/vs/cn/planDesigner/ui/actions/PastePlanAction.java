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

import java.lang.reflect.InvocationTargetException;
import java.util.Collection;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.jface.dialogs.ProgressMonitorDialog;
import org.eclipse.jface.operation.IRunnableWithProgress;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.swt.dnd.Clipboard;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.ui.actions.SelectionListenerAction;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;

public class PastePlanAction extends SelectionListenerAction
{
	public static final String ID = PlanDesignerActivator.PLUGIN_ID + ".CopyPlanAction";
	
	private final Shell shell;

	//private final Clipboard clipboard;

	public PastePlanAction(Shell shell, Clipboard clipboard)
	{
		super("Paste");
	//	this.clipboard = clipboard;
		this.shell = shell;
		setId(ID);
	}
	
	@Override
	public void run()
	{		
		ProgressMonitorDialog dialog = null;
		try {
			dialog = new ProgressMonitorDialog(shell);
		} catch (Exception e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		try {
			dialog.run(true, true, new IRunnableWithProgress()
			{
				public void run(IProgressMonitor monitor)
						throws InvocationTargetException, InterruptedException 
				{
					monitor.beginTask("Counting...", 1000000000);
					for(int i=0; i < 1000000000; i++)
					{
						monitor.worked(1);
						if(monitor.isCanceled())
						{
							break;
						}
					}
					monitor.done();
					
				}
				
			});
		} catch (InvocationTargetException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	protected boolean updateSelection(IStructuredSelection selection) {
		boolean enabled = true;
		Collection<IResource> resources = getSelectedResources();
		for(IResource r : resources)
		{
			enabled = checkResource(r);
			if(!enabled)
			{
				break;
			}
		}
		return enabled;
	}
	
	private boolean checkResource(IResource r) {
		return (r instanceof IFile && ((IFile)r).getFileExtension().equals("pml"));
	}
}
