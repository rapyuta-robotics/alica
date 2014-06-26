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
package de.uni_kassel.vs.cn.planDesigner.ui.properties;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.gef.EditPart;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.ui.IWorkbenchPart;
import org.eclipse.ui.part.IPageSite;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.impl.BehaviourConfigurationImpl;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.SelectionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.views.RepositoryView;

public class PMLTabbedPropertySheetPage extends TabbedPropertySheetPage{
	
	protected ICommandStackTabbedPropertySheetPageContributor commandStackContributor;
	
	public PMLTabbedPropertySheetPage(ICommandStackTabbedPropertySheetPageContributor commandStackContributor) {
		super(commandStackContributor);
		this.commandStackContributor = commandStackContributor;
		
	}
	
	@Override
	public void init(IPageSite pageSite) {
		super.init(pageSite);
		// add some actions to the properties view
//		setPMLEditorActions();
	}
	
	@Override
	public void selectionChanged(IWorkbenchPart part, ISelection selection) {
		// TODO: (DO) If we change selection in the Designer then it causes
		// changing selection in the SourceTab too. We are not going to create
		// PropertySheetPage twice, so, ignore the second case
//		if (selection instanceof ITextSelection) {
//			return;
//		}
		selection = calculateSelection(selection);
		System.out.println(selection);
		
		// Depending on the part we mark the resource read-only or not
		boolean editable = true;
		if(part instanceof RepositoryView)
			editable = false;
		
		PMLTransactionalEditingDomain domain = commandStackContributor.getEditingDomain();
		Map<Resource, Boolean> map = domain.getResourceToReadOnlyMap();
		for(Resource r : domain.getResourceSet().getResources()){
			map.put(r, !editable);
		}
			
		
		super.selectionChanged(part, selection);
		
		// Refresh the current tab to refresh the editable status
		if(getCurrentTab() != null)
			refresh();
	}
	
	
	
	/**
	 * Replace EditPart with model object.
	 */
	protected ISelection calculateSelection(ISelection selection) {
		Set<Object> newSet = new HashSet<Object>();
		if (selection != null && !selection.isEmpty() && (selection instanceof IStructuredSelection)) {
			Iterator<Object> it = ((IStructuredSelection)selection).iterator();
			while (it.hasNext()) {
				Object o = it.next();
				if (o instanceof EditPart) {
					EditPart selectedEditPart = (EditPart)o;
					Object adapter = selectedEditPart.getAdapter(SelectionAdapter.class);
					if(adapter != null)
					{
						o = ((SelectionAdapter)adapter).getAdaptedSelection(selectedEditPart);
					}
					else
					{
						o = selectedEditPart.getModel();
						
					}
				}
				
				newSet.add(o);
			}
		}
		return new StructuredSelection(newSet.toArray(new Object[newSet.size()]));
	}

	protected ICommandStackTabbedPropertySheetPageContributor getCommandStackContributor() {
		return commandStackContributor;
	}

	
}
