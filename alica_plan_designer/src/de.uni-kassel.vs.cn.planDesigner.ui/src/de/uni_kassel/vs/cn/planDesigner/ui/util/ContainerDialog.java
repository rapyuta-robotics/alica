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

import java.util.ArrayList;
import java.util.List;

import org.eclipse.core.internal.resources.WorkspaceRoot;
import org.eclipse.core.resources.IContainer;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IResourceVisitor;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IPath;
import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.ui.ISharedImages;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.ide.IDE.SharedImages;

public class ContainerDialog extends Dialog {
	
	private class ContainerLabelProvider extends LabelProvider{
		@Override
		public Image getImage(Object element) {
			if(element instanceof IContainer){
				ISharedImages imgs = PlatformUI.getWorkbench().getSharedImages();

				IContainer container = (IContainer)element;
				if(container instanceof IProject)
					return imgs.getImage(SharedImages.IMG_OBJ_PROJECT);
				else 
					return imgs.getImage(ISharedImages.IMG_OBJ_FOLDER);
			}else
				return null;
		}
		
		@Override
		public String getText(Object element) {
			if(element instanceof IContainer)
				return ((IContainer)element).getName();
			else 
				return "Unkown"; 
		}
	}

	
	private class ContainerContentProvider implements ITreeContentProvider{

		public Object[] getElements(Object inputElement) {
			if(inputElement instanceof WorkspaceRoot){
				WorkspaceRoot root = (WorkspaceRoot)inputElement;
				Object[] members = null;
				try {
					members = root.members();
				} catch (CoreException e) {
					e.printStackTrace();
				}
				return members;
			}else
				return null;
		}

		public void dispose() {
			
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
			
		}

		public Object[] getChildren(Object parentElement) {
			if(parentElement instanceof IContainer){
				final IContainer container = (IContainer)parentElement;
				final List<IContainer> containers = new ArrayList<IContainer>();
				try {
					container.accept(new IResourceVisitor(){
						public boolean visit(IResource resource)
								throws CoreException {
							if(resource instanceof IContainer){
								if(resource != container)
									containers.add((IContainer)resource);
								return true;
							}else
								return false;
						}
						
					}, IResource.DEPTH_ONE, IResource.NONE);
				} catch (CoreException e) {
					e.printStackTrace();
				}
				return containers.toArray();
			}else
				return null;
		}

		public Object getParent(Object element) {
			if(element instanceof IResource)
				return ((IResource)element).getParent();
			else
				return null;
		}

		public boolean hasChildren(Object element) {
			if(element instanceof IContainer)
				return true;
			else
				return false;
		}
		
	}
	
	private TreeViewer containerViewer;
	
	private ISelection currentSelection;

	public ContainerDialog(Shell parentShell) {
		super(parentShell);
	}
	
	@Override
	protected Control createDialogArea(Composite parent) {
		Composite container = new Composite(parent,SWT.NONE);
		container.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
		container.setLayout(new FillLayout());
		
		containerViewer = new TreeViewer(container);
		containerViewer.setContentProvider(new ContainerContentProvider());
		containerViewer.setLabelProvider(new ContainerLabelProvider());
		
		containerViewer.addSelectionChangedListener(new ISelectionChangedListener(){
			public void selectionChanged(SelectionChangedEvent event) {
				currentSelection = event.getSelection();
			}
		});
		
		containerViewer.setInput(ResourcesPlugin.getWorkspace().getRoot());
		
		
		
		return container;
		
	}
	
	@Override
	protected Point getInitialSize() {
		return new Point(300,400);
	}
	
	public IPath getSelectedPath(){
		IContainer container = (IContainer)((IStructuredSelection)currentSelection).getFirstElement();
		return container.getFullPath();
	}

	
}
