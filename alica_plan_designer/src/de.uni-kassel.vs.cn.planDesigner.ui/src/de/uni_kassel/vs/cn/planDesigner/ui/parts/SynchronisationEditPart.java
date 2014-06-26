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
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.eclipse.draw2d.Figure;
import org.eclipse.draw2d.FlowLayout;
import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.ImageFigure;
import org.eclipse.draw2d.Label;
import org.eclipse.gef.EditPolicy;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.IModelExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.PlanElementExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.SynchedTransitionDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PMLConnectionEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class SynchronisationEditPart extends PlanElementEditPart {
	
	private Figure mainFigure;
	
	private IModelExclusionAdapter modelExclusionAdapter;

	@Override
	protected void addAllAdapters() {
		super.addAllAdapters();
		
		// Register the editPart with the ui extensionmodel
		PmlUiExtension extension = CommonUtils.getUIAwareEditorAdapter(this).getUIExtension(getEObjectModel(), true);
		if(extension != null)
			this.adapter.addToObject(extension);
	}
	
	@Override
	public Object getAdapter(Class key) {
		if(key == IModelExclusionAdapter.class){
			return getModelExclusionAdapter();
		}
		return super.getAdapter(key);
	}

	private IModelExclusionAdapter getModelExclusionAdapter() {
		if(this.modelExclusionAdapter == null){
			this.modelExclusionAdapter = new PlanElementExclusionAdapter(){
				@Override
				protected Set<String> createExclusionClasses() {
					Set<String> ex = new HashSet<String>();
					ex.add(AlicaPackage.eINSTANCE.getTransition().getName());
					return ex;
				}
			};
		}
		return this.modelExclusionAdapter;
	}

	@Override
	protected IFigure createFigure() {		
		IFigure wrapper = new Figure();
		FlowLayout fl = new FlowLayout(false);
		fl.setMajorSpacing(0);
		fl.setMinorSpacing(0);
		fl.setMinorAlignment(FlowLayout.ALIGN_CENTER);
		wrapper.setLayoutManager(fl);
		
		PlanDesignerActivator plugin = PlanDesignerActivator.getDefault();
		
		this.mainFigure = new ImageFigure(plugin.getImageRegistry().get(PlanDesignerConstants.ICON_SYNCHRONISATION_36));

		wrapper.add(getNameLabel());
		wrapper.add(this.mainFigure);
		
		return wrapper;
	}
	
	@Override
	public IFigure getMainFigure() {
		return this.mainFigure;
	};
	
	@Override
	protected List<SynchedTransitionDummyConnection> getModelSourceConnections() {
		List<SynchedTransitionDummyConnection> connections = null;
		if (getSynchronisation().getSynchedTransitions() != null && getSynchronisation().getSynchedTransitions().size() > 0) {
			connections = new ArrayList<SynchedTransitionDummyConnection>();
			for (Transition trans : getSynchronisation().getSynchedTransitions()) {				
				connections.add( new SynchedTransitionDummyConnection(getSynchronisation(), trans) );
			}
		} else {
			connections = Collections.emptyList();
		}
		return connections;
	}
	
	protected Synchronisation getSynchronisation(){
		return (Synchronisation)getModel();
	}
	
	@Override
	protected void createEditPolicies () {
		super.createEditPolicies();
		installEditPolicy(EditPolicy.GRAPHICAL_NODE_ROLE, new PMLConnectionEditPolicy());
	}
	
	@Override
	protected Label createNameLabel() {
		return new Label();
	}
	
//	@Override
//	protected List getModelChildren() {
//		List<SynchedTransitionDummyConnection> connections = null;
//		if (getSynchronisation().getSynchedTransitions() != null && getSynchronisation().getSynchedTransitions().size() > 0) {
//			connections = new ArrayList<SynchedTransitionDummyConnection>();
//			for (Transition trans : getSynchronisation().getSynchedTransitions()) {				
//				connections.add( new SynchedTransitionDummyConnection(getSynchronisation(), trans) );
//			}
//		} else {
//			connections = Collections.emptyList();
//		}
//		return connections;
//	}

}
