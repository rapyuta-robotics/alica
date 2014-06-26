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
package de.uni_kassel.vs.cn.planDesigner.ui.parts;

import java.io.IOException;

import org.eclipse.core.resources.IFile;
import org.eclipse.draw2d.Label;
import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.transaction.NotificationFilter;
import org.eclipse.emf.transaction.ResourceSetChangeEvent;
import org.eclipse.emf.transaction.ResourceSetListener;
import org.eclipse.emf.transaction.RollbackException;
import org.eclipse.emf.workspace.util.WorkspaceSynchronizer;
import org.eclipse.gef.DragTracker;
import org.eclipse.gef.Request;
import org.eclipse.gef.tools.DragEditPartsTracker;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.ide.IDE;

import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public class PlanStateEditPart extends AbstractPlanStateEditPart {
	
	private ResourceSetListener resourceSetListener;
	
	private Resource resource;
	
	@Override
	public void activate() {
		super.activate();
//		setResource(getEObjectModel().eResource());
		
		//TransactionalEditingDomain editingDomainAdapter = PlanEditorUtils.getEditingDomainAdapter(this);
//		editingDomainAdapter.addResourceSetListener(getResourceSetListener());
	}
	
	@Override
	public void deactivate() {
		super.deactivate();
		//TransactionalEditingDomain editingDomainAdapter = PlanEditorUtils.getEditingDomainAdapter(this);
//		editingDomainAdapter.removeResourceSetListener(getResourceSetListener());
	}
	@Override
	protected Label createNameLabel() {
		Label l = new Label(getPlanElement().getName());
		
		PlanDesignerActivator plugin = PlanDesignerActivator.getDefault();
		
		l.setIcon(plugin.getImageRegistry().get(PlanDesignerConstants.ICON_PLAN_16));
		
		return l;
	}
	
	@Override
	public DragTracker getDragTracker(Request request) {
		return new DragEditPartsTracker(this){

			@Override
			protected boolean handleDoubleClick(int button) {
				if(button == 1){
//					// Open the nested plan in a new editor
					EObject model = getEObjectModel();
					
					// TODO: Fix this to not reload the file
					// here but as a result of some resourceSetChange event
					if(model.eIsProxy()){
						model = EcoreUtil.resolve(model, 
								PlanEditorUtils.getEditingDomainAdapter(PlanStateEditPart.this).getResourceSet());
						setModel(model);
						refreshAdapters();
						refresh();
					}
//					
					IFile file = WorkspaceSynchronizer.getFile(model.eResource());
					PlanEditor editor = PlanEditorUtils.getPlanEditor(PlanStateEditPart.this);
					try {
						IDE.openEditor(editor.getSite().getPage(), file);
					} catch (PartInitException e) {
						e.printStackTrace();
					}
				}
				return true;
			}
			
		};
	}

	public void setResourceSetListener(ResourceSetListener resourceSetListener) {
		this.resourceSetListener = resourceSetListener;
	}

	public ResourceSetListener getResourceSetListener() {
		if(resourceSetListener == null)
		{
			resourceSetListener = new ResourceSetListener(){

				public NotificationFilter getFilter() {
					return NotificationFilter.RESOURCE_UNLOADED;
				}

				public boolean isAggregatePrecommitListener() {
					// TODO Auto-generated method stub
					return false;
				}

				public boolean isPostcommitOnly() {
					// TODO Auto-generated method stub
					return false;
				}

				public boolean isPrecommitOnly() {
					// TODO Auto-generated method stub
					return false;
				}

				public void resourceSetChanged(ResourceSetChangeEvent event) {
					 if(getEObjectModel().eResource() == null)
					 {
						 reloadResource();
					 }
				}

				public Command transactionAboutToCommit(ResourceSetChangeEvent event)
						throws RollbackException {
					// TODO Auto-generated method stub
					return null;
				}
				
			};
		}
		return resourceSetListener;
	}
	
	private void reloadResource() {
		Resource res = getResource();
		
		try {
			res.load(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		setModel(res.getContents().get(0));
		refreshAdapters();
		refresh();
	}

	protected Resource getResource() {
		return resource;
	}

	protected void setResource(Resource resource) {
		this.resource = resource;
	}
}
