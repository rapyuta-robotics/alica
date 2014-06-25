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

import java.util.Iterator;

import org.eclipse.draw2d.LightweightSystem;
import org.eclipse.draw2d.MarginBorder;
import org.eclipse.draw2d.Viewport;
import org.eclipse.draw2d.parts.ScrollableThumbnail;
import org.eclipse.draw2d.parts.Thumbnail;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.transaction.ResourceSetChangeEvent;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.LayerConstants;
import org.eclipse.gef.editparts.ScalableFreeformRootEditPart;
import org.eclipse.gef.ui.parts.ContentOutlinePage;
import org.eclipse.jface.action.Action;
import org.eclipse.jface.action.IToolBarManager;
import org.eclipse.jface.resource.ImageDescriptor;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Canvas;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Table;
import org.eclipse.ui.IWorkbenchPart;
import org.eclipse.ui.part.PageBook;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.ContentChangeListener;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.HiddenElementProviderFactory;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.ExecuteAndRefreshPlanCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.HideElementCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;

public class OverviewOutlinePage extends ContentOutlinePage implements ContentChangeListener{

	/** the control of the overview */
	private Canvas overview;

	/** the thumbnail */
	private Thumbnail thumbnail;
	
	private PageBook pageBook;
	
	private TableViewer hiddenElementsViewer;

	private final IWorkbenchPart part;

	private Composite hiddenElementsHolder;

	/**
	 * Creates a new OverviewOutlinePage instance.
	 * 
	 * @param rootEditPart
	 *            the root edit part to show the overview from
	 */
	public OverviewOutlinePage(IWorkbenchPart part, EditPartViewer viewer) {
		super(viewer);
		this.part = part;
	}

	public void createControl(Composite parent) {
		pageBook = new PageBook(parent, SWT.NONE);
		
		// create canvas and lws
		overview = new Canvas(pageBook, SWT.NONE);
		LightweightSystem lws = new LightweightSystem(overview);
		// create thumbnail
		ScalableFreeformRootEditPart rootEditPart = (ScalableFreeformRootEditPart)getViewer().getRootEditPart();
		Viewport viewport = (Viewport)rootEditPart.getFigure();
		thumbnail = new ScrollableThumbnail(viewport);

		thumbnail.setBorder(new MarginBorder(3));
		thumbnail.setSource(rootEditPart.getLayer(LayerConstants.PRINTABLE_LAYERS));
		lws.setContents(thumbnail);
		
		hiddenElementsHolder = new Composite(pageBook, SWT.NONE);
		GridLayout layout = new GridLayout();
		layout.marginWidth = 0;
		layout.marginHeight = 0;
		layout.verticalSpacing = 0;
		hiddenElementsHolder.setLayout(layout);
		
		Label description = new Label(hiddenElementsHolder, SWT.NONE);
		description.setBackground(new Color(null,192, 192, 192));
		description.setText("Double-click to unhide");
		
		GridData layoutData = new GridData(SWT.FILL,SWT.FILL,true,false);
		layoutData.heightHint = 20;
		description.setLayoutData(layoutData);
		
		Table table = new Table(hiddenElementsHolder, SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL | SWT.BORDER | SWT.FULL_SELECTION);
		table.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
		
		// Create the viewer for the hidden elements
		hiddenElementsViewer = new TableViewer(table);
		hiddenElementsViewer.addDoubleClickListener(new IDoubleClickListener()
		{

			public void doubleClick(DoubleClickEvent event)
			{
				IStructuredSelection selection = (IStructuredSelection)hiddenElementsViewer.getSelection();
				if(!selection.isEmpty())
				{
					Object adapter = part.getAdapter(UIAwareEditor.class);
					if(adapter != null)
					{
						UIAwareEditor editor = (UIAwareEditor)adapter;
						
						PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
								PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
						
						CompoundCommand cmp = new CompoundCommand("Hide element");
						
						for(Iterator<Object> partIterator = selection.iterator(); partIterator.hasNext();)
						{
							final Object object = partIterator.next();
							if(object instanceof EObject)
							{
								cmp.append(new HideElementCommand(editingDomain, editor, (EObject)object, false));
							}

						}
						
						// Add a command which will refresh the plan
						ExecuteAndRefreshPlanCommand cmd = new ExecuteAndRefreshPlanCommand(cmp, editor.getRootEditPart());
						editingDomain.getCommandStack().execute(cmd);	
					}
					
				}
				
			}
			
		});
		Object adapter = part.getAdapter(HiddenElementProviderFactory.class);
		if(adapter != null)
		{
			HiddenElementProviderFactory hiddenElementProviderFactory = (HiddenElementProviderFactory)adapter;
			hiddenElementProviderFactory.addContentChangeListener(this);
			hiddenElementsViewer.setContentProvider(hiddenElementProviderFactory.getHiddenElementContentProvider());
			hiddenElementsViewer.setLabelProvider(hiddenElementProviderFactory.getHiddenElementLabelProvider());
			hiddenElementsViewer.setInput(hiddenElementProviderFactory.getInput());
		}
		
		pageBook.showPage(overview);
		
		configureToolbar();
	}
	
	private void configureToolbar()
	{
		IToolBarManager tbm = getSite().getActionBars().getToolBarManager();
		
		tbm.add(new Action("Show overview", 
				ImageDescriptor.createFromImage(
						PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_OVERVIEW_16)))
		{
			@Override
			public void run()
			{
				showPage(overview);
			}
		});
		
		tbm.add(new Action("Show hidden elements", 
		ImageDescriptor.createFromImage(
				PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_HIDDEN_16)))
		{
			@Override
			public void run()
			{
				showPage(hiddenElementsHolder);
			}
		});
	}
	
	private void showPage(Control page)
	{
		pageBook.showPage(page);
	}

	public void dispose() {
		Object adapter = part.getAdapter(HiddenElementProviderFactory.class);
		if(adapter != null)
		{
			HiddenElementProviderFactory hiddenElementProviderFactory = (HiddenElementProviderFactory)adapter;
			hiddenElementProviderFactory.removeContentChangeListener(this);
		}
		
		if (null != thumbnail)
			thumbnail.deactivate();
		
		super.dispose();
	}

	public Control getControl() {
		return pageBook;
	}

	public void setFocus() {
		if (getControl() != null)
			getControl().setFocus();
	}

	public void contentChanged(ResourceSetChangeEvent event)
	{
		hiddenElementsViewer.refresh();
	}

}
