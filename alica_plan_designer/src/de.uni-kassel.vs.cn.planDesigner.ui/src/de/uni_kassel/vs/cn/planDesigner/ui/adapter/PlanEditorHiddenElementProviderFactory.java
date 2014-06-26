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
package de.uni_kassel.vs.cn.planDesigner.ui.adapter;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.transaction.ResourceSetChangeEvent;
import org.eclipse.emf.transaction.ResourceSetListener;
import org.eclipse.emf.transaction.ResourceSetListenerImpl;
import org.eclipse.jface.resource.ImageRegistry;
import org.eclipse.jface.viewers.IContentProvider;
import org.eclipse.jface.viewers.ILabelProvider;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.swt.graphics.Image;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class PlanEditorHiddenElementProviderFactory implements
		HiddenElementProviderFactory
{

	private class HiddenElementContentProvider implements ITreeContentProvider
	{

		private final PlanEditor planEditor;

		public HiddenElementContentProvider(PlanEditor editor)
		{
			planEditor = editor;
		}
		
		public Object[] getElements(Object inputElement)
		{
			Collection<PlanElement> hiddenElements = new ArrayList<PlanElement>();
			
			if(inputElement instanceof Plan)
			{
				Plan p = (Plan)inputElement;
				for (State s : p.getStates())
				{
					if(!CommonUtils.isVisible(planEditor, s))
					{
						hiddenElements.add(s);
					}
				}
			}
			
			if(hiddenElements.isEmpty())
			{
				return new Object[]{"No hidden elements"};
			}
			else
			{
				return hiddenElements.toArray();
			}
		}
		
		public void dispose()
		{
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput)
		{
		}

		public Object[] getChildren(Object parentElement)
		{
			return null;
		}

		public Object getParent(Object element)
		{
			return null;
		}

		public boolean hasChildren(Object element)
		{
			return false;
		}
	}
	
	private class HiddenElementLabelProvider extends LabelProvider
	{

		public Image getImage(Object element)
		{
			ImageRegistry imageRegistry = PlanDesignerActivator.getDefault().getImageRegistry();
			
			Image img = imageRegistry.
						get(PlanDesignerConstants.ICON_UNKNOWN_TYPE);
				
			if(element instanceof State)
			{
				img = imageRegistry.
						get(PlanDesignerConstants.ICON_STATE_16);
			}
			
			return img;
		}

		public String getText(Object element)
		{
			String text = element.toString();
			
			if(element instanceof PlanElement)
			{
				text = ((PlanElement)element).getName(); 
			}
			
			return text;
		}
	}
	
	private class HiddenElementHelper extends ResourceSetListenerImpl
	{
		@Override
		public void resourceSetChanged(ResourceSetChangeEvent event)
		{
			boolean fire = false;
			
			for (Notification n : event.getNotifications())
			{
				if(n.getFeature() == PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Visible())
				{
					fire = true;
					break;
				}
			}

			if(fire)
			{
				fireContentChange(event);
			}
		}
	}
	
	private final PlanEditor editor;
	
	private IContentProvider contentProvider;
	
	private ILabelProvider labelProvider;
	
	private ResourceSetListener hiddenElementHelper;
	
	private List<ContentChangeListener> contentChangeListeners = new ArrayList<ContentChangeListener>();

	public PlanEditorHiddenElementProviderFactory(PlanEditor editor)
	{
		this.editor = editor;
		
		this.editor.getEditingDomain().addResourceSetListener(getHiddenElementHelper());
	}
	
	public IContentProvider getHiddenElementContentProvider()
	{
		if(contentProvider == null)
		{
			contentProvider = new HiddenElementContentProvider(editor);
		}
		
		return contentProvider;
	}

	public ILabelProvider getHiddenElementLabelProvider()
	{
		if(labelProvider == null)
		{
			labelProvider = new HiddenElementLabelProvider();
		}
		
		return labelProvider;
	}

	public Object getInput()
	{
		return editor.getPlan();
	}

	public void dispose()
	{
		editor.getEditingDomain().removeResourceSetListener(getHiddenElementHelper());
	}
	
	private ResourceSetListener getHiddenElementHelper()
	{
		if(hiddenElementHelper == null)
		{
			hiddenElementHelper = new HiddenElementHelper();
		}
		
		return hiddenElementHelper;
	}
	
	private void fireContentChange(ResourceSetChangeEvent event)
	{
		for (ContentChangeListener listener : contentChangeListeners)
		{
			listener.contentChanged(event);
		}
	}

	public void addContentChangeListener(ContentChangeListener listener)
	{
		if(listener != null)
		{
			contentChangeListeners.add(listener);
		}
		else
		{
			throw new IllegalArgumentException("listener cannot be null!");
		}
	}

	public boolean removeContentChangeListener(ContentChangeListener listener)
	{
		if(listener != null)
		{
			return contentChangeListeners.remove(listener);
		}
		else
		{
			throw new IllegalArgumentException("listener cannot be null!");
		}
	}
	

}
