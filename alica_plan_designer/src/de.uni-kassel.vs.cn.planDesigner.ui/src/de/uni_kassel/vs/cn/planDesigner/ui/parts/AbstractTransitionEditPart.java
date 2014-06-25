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

import java.util.ArrayList;
import java.util.List;

import org.eclipse.draw2d.AbsoluteBendpoint;
import org.eclipse.draw2d.ColorConstants;
import org.eclipse.draw2d.Graphics;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.Label;
import org.eclipse.draw2d.PolygonDecoration;
import org.eclipse.draw2d.PolylineConnection;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.editparts.AbstractConnectionEditPart;
import org.eclipse.gef.editpolicies.ConnectionEndpointEditPolicy;
import org.eclipse.swt.SWT;
import org.eclipse.ui.views.properties.IPropertySource;
import org.eclipse.ui.views.properties.IPropertySourceProvider;

import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.Bendpoint;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.MultiObjectNotificationAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.PropertySourceAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;

public abstract class AbstractTransitionEditPart extends
		AbstractConnectionEditPart implements IPropertySourceProvider {

	private Label nameLabel;
	
	/**
	 * The adapter receives notifications from the model
	 */
	protected MultiObjectNotificationAdapter adapter;

	public AbstractTransitionEditPart() {
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

	protected void handleModelChanged(Notification n) {
		if(n.getNotifier() instanceof PmlUiExtension || n.getNotifier() instanceof Bendpoint){
			refreshUIExtension();
		}else{
			int type = n.getEventType();
			switch(type){
			case Notification.ADD:
			case Notification.ADD_MANY:
			case Notification.REMOVE:
			case Notification.REMOVE_MANY:
				refresh();
				break;
			case Notification.SET:
				refreshVisuals();
				break;
			}
		}
	}

	protected void refreshUIExtension() {
		refreshBendpoints();
	}

	protected void addAllAdapters() {
		// Register the editPart with the model
		this.adapter.addToObject((EObject)getModel());
	}
	
	protected PlanElement getPlanElementModel(){
		return (PlanElement)getModel();
	}

	protected void refreshAdapters() {
		this.adapter.removeFromObjects();
		addAllAdapters();
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

	@Override
	protected void refreshVisuals() {
		getNameLabel().setText(getPlanElementModel().getName());
	}

	protected List<org.eclipse.draw2d.Bendpoint> getRoutingConstraint() {
		List<Bendpoint> bendpoints = CommonUtils.getUIAwareEditorAdapter(this).getUIExtension(getPlanElementModel(), true).getBendpoints();
		List<org.eclipse.draw2d.Bendpoint> routingConstraint = new ArrayList<org.eclipse.draw2d.Bendpoint>();
		for(Bendpoint bp : bendpoints){
			routingConstraint.add(new AbsoluteBendpoint(bp.getXPos(),bp.getYPos()));
		}
		
		return routingConstraint;
	}

	private void refreshBendpoints() {
		getConnectionFigure().setRoutingConstraint(getRoutingConstraint());
	}

	@Override
	protected void createEditPolicies() {
			installEditPolicy(EditPolicy.CONNECTION_ENDPOINTS_ROLE, new ConnectionEndpointEditPolicy());
			// TODO: Install a BendpointEditPolicy!
			
			// The Component EditPolicy is not needed here because we use the EMF delete
			// action to delete object. But we may forward the command creation to this
			// policy later if we decide that just using the action wasn't a good choice
	//		installEditPolicy(EditPolicy.COMPONENT_ROLE, new TransitionEditPolicy());
		}

	public Label getNameLabel() {
		if(this.nameLabel == null){
			this.nameLabel = new Label(getPlanElementModel().getName());
			this.nameLabel.setBackgroundColor(ColorConstants.white);
			this.nameLabel.setOpaque(true);
		}		
		return nameLabel;
	}

	/**
	 * Give the given connection the opportunity to get configured. This can 
	 * be the definition of colors or decoration for example. There is already a
	 * connectionRouter applied on the given connection.
	 * @param con
	 */
	protected abstract void configureConnection(PolylineConnection con);

	public IPropertySource getPropertySource(Object object) {
		return new PropertySourceAdapter(getPlanElementModel());
	}

	@Override
	protected IFigure createFigure() {
		PolylineConnection con = new PolylineConnection(){
			@Override
			public void paint(Graphics graphics) {
				graphics.setAntialias(SWT.ON);
				super.paint(graphics);
			}
		};
		PolygonDecoration arrow = new PolygonDecoration();
		arrow.setTemplate(PolygonDecoration.TRIANGLE_TIP);
		con.setTargetDecoration(arrow);
		
		con.setLineWidth(2);
		
		// Hook for subclasses to further modify the connection
		configureConnection(con);
		
		return con;
	}

}