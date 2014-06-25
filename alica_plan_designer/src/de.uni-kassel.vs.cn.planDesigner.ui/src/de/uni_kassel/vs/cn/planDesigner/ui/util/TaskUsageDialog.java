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
package de.uni_kassel.vs.cn.planDesigner.ui.util;

import java.util.Set;

import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Shell;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;

public class TaskUsageDialog extends MessageDialog {
	
	private class PlanContentProvider implements IStructuredContentProvider{

		public Object[] getElements(Object inputElement) {
			Object[] result = new String[]{"No usages found!"};
			// We expect a set of plans to display
			if(inputElement instanceof Set)
			{
				if(!((Set)inputElement).isEmpty())
				{
					result = ((Set)inputElement).toArray();
				}
			}
			
			return result;
		}

		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
		}
	}
	
	private class PlanLabelProvider extends LabelProvider{
		@Override
		public Image getImage(Object element) {
			Image img;
			if(element instanceof Plan)
			{
				if(((Plan)element).isMasterPlan())
				{
					img = PlanDesignerActivator.getDefault().getImageRegistry().get(
							PlanDesignerConstants.ICON_MASTER_PLAN_16);
				}
				else
				{
					img = PlanDesignerActivator.getDefault().getImageRegistry().get(
							PlanDesignerConstants.ICON_PLAN_16);
				}
			}
			else
			{
				img = PlanDesignerActivator.getDefault().getImageRegistry().get(
						PlanDesignerConstants.ICON_UNKNOWN_TYPE);
			}
				
			return img;
		}
		
		@Override
		public String getText(Object element) {
			if(element instanceof Plan)
				return ((Plan)element).getName();
			else
				return element.toString();
		}
	}

	private Set<Plan> affectedPlans;
	
	public TaskUsageDialog(Shell parent, Set<Plan> affectedPlans, String title, String message, int dialogImageType) {
		super(parent, title, null, // accept
                // the
                // default
                // window
                // icon
				message, dialogImageType, new String[] { IDialogConstants.OK_LABEL, IDialogConstants.CANCEL_LABEL }, 0);
		this.affectedPlans = affectedPlans;
	}
	
	@Override
	protected Control createCustomArea(Composite parent) {
		Composite comp = new Composite(parent, SWT.BORDER);
		comp.setLayout(new FillLayout());
		comp.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
		
		TableViewer viewer = new TableViewer(comp);
		viewer.setContentProvider(new PlanContentProvider());
		viewer.setLabelProvider(new PlanLabelProvider());
		viewer.setInput(affectedPlans);

		return comp;
	}
	
	@Override
	protected Point getInitialSize() {
		return new Point(500,350);
	}
}
