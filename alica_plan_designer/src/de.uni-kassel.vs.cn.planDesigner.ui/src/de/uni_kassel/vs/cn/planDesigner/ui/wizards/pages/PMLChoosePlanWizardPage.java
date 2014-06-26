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
package de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.core.resources.IContainer;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IFolder;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.jface.dialogs.IDialogPage;
import org.eclipse.jface.dialogs.IMessageProvider;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.ui.ISharedImages;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.ide.IDE.SharedImages;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

/**
 * The "New" wizard page allows setting the container for the new file as well
 * as the file name. The page will only accept file name without the extension
 * OR with the extension that matches the expected one (pml).
 */

public class PMLChoosePlanWizardPage extends WizardPage {
	
	private class WorkspaceContentProvider implements ITreeContentProvider{

		public Object[] getElements(Object inputElement) {
			IWorkspaceRoot root = (IWorkspaceRoot)inputElement;
			List<IContainer> container = new ArrayList<IContainer>();
			try {
				for(IResource r : root.members()){
					if(r instanceof IContainer){
						container.add((IContainer)r);
					}
				}
			} catch (CoreException e) {
				e.printStackTrace();
			}
			return container.toArray();
		}

		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
		}

		public Object[] getChildren(Object parentElement) {
			if(parentElement instanceof IContainer){
				List<IResource> container = new ArrayList<IResource>();
				try {
					for(IResource r : ((IContainer)parentElement).members()){
						if(!(r instanceof IContainer)){
							// Add the file only if it is a plan
							String ext = r.getFileExtension();
							if(ext != null && ext.equals("pml"))
								container.add(r);
						}else if(!r.getName().startsWith("."))
							container.add(r);
					}
				} catch (CoreException e) {
					e.printStackTrace();
				}
				return container.toArray();
			}else
				return new Object[0];
		}

		public Object getParent(Object element) {
			if(element instanceof IResource)
				return ((IResource)element).getParent();
			else
				return null;
		}

		public boolean hasChildren(Object element) {
			if(element instanceof IContainer){
				try {
					return ((IContainer)element).members().length > 0;
				} catch (CoreException e) {
					e.printStackTrace();
				}
			}
			return false;
		}
	}
	
	private class WorkspaceLabelProvider extends LabelProvider{
		@Override
		public String getText(Object element) {
			if(element instanceof IResource){
				return ((IResource)element).getName();
			}else
				return "UNKNOWN";
			
		}
		
		@Override
		public Image getImage(Object element) {
			ISharedImages images = PlatformUI.getWorkbench().getSharedImages();
			Image img = null;
			if(element instanceof IProject){
				img = images.getImage(SharedImages.IMG_OBJ_PROJECT);
			}else if(element instanceof IFolder){
				img = images.getImage(ISharedImages.IMG_OBJ_FOLDER);
			}else if(element instanceof IFile){
				img = PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_PLAN_16);
			}
			
				
			
			return img;
		}
	}

	private TreeViewer workspaceViewer;
	
	private ISelection currentSelection;

	public PMLChoosePlanWizardPage() {
		super("wizardPage");
		setTitle("Choose plan");
		setDescription("Choose the master plan for which you want to create a roleset.");
	}

	/**
	 * @see IDialogPage#createControl(Composite)
	 */
	public void createControl(Composite parent) {
		Composite client = new Composite(parent, SWT.NONE);
		client.setLayout(new FillLayout());
		
		workspaceViewer = new TreeViewer(client);
		workspaceViewer.setContentProvider(new WorkspaceContentProvider());
		workspaceViewer.setLabelProvider(new WorkspaceLabelProvider());
		workspaceViewer.addSelectionChangedListener(new ISelectionChangedListener(){
			public void selectionChanged(SelectionChangedEvent event) {
				currentSelection = event.getSelection();
				dialogChanged();
			}
		});
		
		workspaceViewer.setInput(ResourcesPlugin.getWorkspace().getRoot());
		
		dialogChanged();
		
		setControl(client);
	}
	
	public IResource getSelectedResource(){
		IResource resource = null;
		if(currentSelection != null)
			resource = (IResource)((IStructuredSelection)currentSelection).getFirstElement();

		return resource;
	}
	
	private void dialogChanged() {
		IResource selectedFile = getSelectedResource();
		
		if(selectedFile == null || selectedFile.getFileExtension() == null || !selectedFile.getFileExtension().equals("pml")){
			updateStatus("Select a plan!");
			return;
		}
			
		updateStatus(null);
	}
	
	private void updateStatus(String message){
		updateStatus(message, IMessageProvider.ERROR);
	}

	private void updateStatus(String message, int type) {
		setMessage(message, type);
		
		setPageComplete(message == null || type != IMessageProvider.ERROR);
	}
}