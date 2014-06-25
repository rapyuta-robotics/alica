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

import org.eclipse.core.runtime.IAdaptable;
import org.eclipse.draw2d.ConnectionAnchor;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.XYLayout;
import org.eclipse.draw2d.geometry.Rectangle;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EReference;
import org.eclipse.gef.ConnectionEditPart;
import org.eclipse.gef.NodeEditPart;
import org.eclipse.gef.Request;
import org.eclipse.gef.editparts.AbstractGraphicalEditPart;
import org.eclipse.ui.views.properties.IPropertySource;
import org.eclipse.ui.views.properties.IPropertySourceProvider;

import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.MultiObjectNotificationAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.PropertySourceAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;

/**
 * Abstract base editPart which has an MultiObjectNotificationAdapter. This adapter can 
 * be registered on EObjects to receive model notifications. 
 * This EditPart knows how to handle notifications from UI Extensions, however the adapter
 * has to registered on an UI Extension in addAllAdapters() method.
 * @author Zenobios
 *
 */
public abstract class AbstractElementEditPart extends AbstractGraphicalEditPart
		implements NodeEditPart, IAdaptable, IPropertySourceProvider {
	
	/**
	 * The adapter receives notifications from the model
	 */
	protected MultiObjectNotificationAdapter adapter;
	
	public AbstractElementEditPart() {
		super();

		this.adapter = new MultiObjectNotificationAdapter(){
			@Override
			public void doNotify(Notification notification) {
				// Forward the notification
				handleModelChanged(notification);
				
				// Refresh the adapters
				refreshAdapters();
			}
		};
	}

	protected void handleModelChanged(Notification n){
		int type = n.getEventType();
		switch(type){
		case Notification.ADD:
		case Notification.ADD_MANY:
		case Notification.REMOVE:
		case Notification.REMOVE_MANY:
			refresh();
			break;
		case Notification.SET:
			// In case we have a set but the feature is a reference
			// we have to update our structural children
			if(n.getFeature() instanceof EReference)
				refresh();
			else if(n.getNotifier() instanceof PmlUiExtension)
				refreshUIExtension(n);
			else
				refreshVisuals();
			break;
		}
		
	}
	
	protected void addAllAdapters(){
		// Register the editPart with the model
		this.adapter.addToObject((EObject)getModel());
	}
	
	protected void refreshAdapters(){
		this.adapter.removeFromObjects();
		addAllAdapters();
	}
	
	protected void refreshUIExtension(Notification n){
		int feature = n.getFeatureID(null);
		
		switch(feature){
		case PmlUIExtensionModelPackage.PML_UI_EXTENSION__XPOS:
		case PmlUIExtensionModelPackage.PML_UI_EXTENSION__YPOS:
		case PmlUIExtensionModelPackage.PML_UI_EXTENSION__WIDTH:
		case PmlUIExtensionModelPackage.PML_UI_EXTENSION__HEIGHT:
			// Get the right figure, which holds the layout constraints for this editParts figure
			IFigure parentFigure = null;
			if(getParent() instanceof PlanElementEditPart){
				parentFigure = ((PlanElementEditPart)getParent()).getMainFigure();
			}else
				parentFigure = ((AbstractGraphicalEditPart)getParent()).getFigure();
			
			if(parentFigure.getLayoutManager() instanceof XYLayout){
				// Get the extension 
				PmlUiExtension ext = CommonUtils.getUIAwareEditorAdapter(this).getUIExtension(getEObjectModel(),true);
				
				if(ext != null){
					parentFigure.getLayoutManager().setConstraint(getFigure(),new Rectangle(ext.getXPos(),ext.getYPos(),ext.getWidth(),ext.getHeight()));
					parentFigure.revalidate();
				}
			}
			break;
		}
	}
	
	public IPropertySource getPropertySource(Object object) {
		System.out.println(object);
		return new PropertySourceAdapter(getEObjectModel());
	}
	
	public EObject getEObjectModel(){
		return (EObject)getModel();
	}
	
	/**
	 * Returns the main figure for this editPart. By default it simply returns getFigure().
	 * This is useful if this editParts figure is complex.
	 * @return
	 */
	public IFigure getMainFigure(){
		return getFigure();
	}
	
	@Override
	public void activate() {
		if(isActive())
			return;
		
		addAllAdapters();
		
		super.activate();
	}

	@Override
	public void deactivate() {
		super.deactivate();
		
		this.adapter.removeFromObjects();
	}

	public ConnectionAnchor getSourceConnectionAnchor(
			ConnectionEditPart connection) {
		return null;
	}

	public ConnectionAnchor getSourceConnectionAnchor(Request request) {
		return null;
	}

	public ConnectionAnchor getTargetConnectionAnchor(
			ConnectionEditPart connection) {
		return null;
	}

	public ConnectionAnchor getTargetConnectionAnchor(Request request) {
		return null;
	}
}
