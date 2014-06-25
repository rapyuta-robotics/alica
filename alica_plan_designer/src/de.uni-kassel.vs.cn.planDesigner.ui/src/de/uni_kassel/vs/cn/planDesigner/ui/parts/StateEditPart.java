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
import org.eclipse.draw2d.geometry.Dimension;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.util.EList;
import org.eclipse.gef.EditPolicy;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.IModelExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.PlanElementExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.EntryPointStateDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.SynchedTransitionDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.DragDropEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PMLConnectionEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PMLDirectEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PMLFlowLayoutEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public class StateEditPart extends CollapsableEditPart{
	
	public static final String DRAG_DROP_ROLE = "DragDropRole";

	private Figure mainFigure;
	
	private IModelExclusionAdapter modelExclusionAdapter;
	
	@Override
	protected void addAllAdapters() {
		super.addAllAdapters();
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
					ex.add(AlicaPackage.eINSTANCE.getEntryPoint().getName());
					ex.add(AlicaPackage.eINSTANCE.getSuccessState().getName());
					ex.add(AlicaPackage.eINSTANCE.getFailureState().getName());
					ex.add(AlicaPackage.eINSTANCE.getSynchronisation().getName());
					ex.add(AlicaPackage.eINSTANCE.getPostCondition().getName());
					ex.add(SynchedTransitionDummyConnection.class.getName());
					return ex;
				}
			};
		}
		return this.modelExclusionAdapter;
	}
	
	@Override
	protected IFigure createDescriptionFigure() {
		IFigure wrapper = new Figure();
		FlowLayout fl = new FlowLayout(false);
		fl.setMajorSpacing(0);
		fl.setMinorSpacing(0);
		fl.setMinorAlignment(FlowLayout.ALIGN_CENTER);
		wrapper.setLayoutManager(fl);
		
		PlanDesignerActivator plugin = PlanDesignerActivator.getDefault();
		
		this.mainFigure = new ImageFigure(plugin.getImageRegistry().get(PlanDesignerConstants.ICON_STATE_50));

		wrapper.add(getNameLabel());
		wrapper.add(this.mainFigure);
		
		return wrapper;
	}
	
	@Override
	protected Label createNameLabel() {
		Label label = new Label(getPlanElement().getName());
		label.setPreferredSize(new Dimension(50,-1));
		return label;
	}
	
	@Override
	protected void createEditPolicies() {
		super.createEditPolicies();
		installEditPolicy(EditPolicy.LAYOUT_ROLE, new PMLFlowLayoutEditPolicy());
		installEditPolicy(EditPolicy.GRAPHICAL_NODE_ROLE, new PMLConnectionEditPolicy());
		installEditPolicy(DRAG_DROP_ROLE, new DragDropEditPolicy());
	}
	
	
	@Override
	public IFigure getMainFigure() {
		return this.mainFigure;
	}

	@Override
	protected List<Transition> getModelSourceConnections() {
		List<Transition> sources = null;
		EList<Transition> outTransition = ((State) getModel()).getOutTransitions();
		if (outTransition != null) 
		{
			sources = new ArrayList<Transition>();
			for (Transition t : outTransition) 
			{
				if (CommonUtils.isVisible(PlanEditorUtils.getPlanEditor(this),
						t.getOutState())) {
					sources.add(t);
				}
			}

		}
		else
		{
			sources = Collections.emptyList();
		}
		return sources;
	}

	@Override
	protected List<Object> getModelTargetConnections() {
		List<Object> targets = new ArrayList<Object> ();
		EList<Transition> inTransition = ((State)getModel()).getInTransitions();
		if(inTransition != null)
		{
			for(Transition t : inTransition)
			{
				if(CommonUtils.isVisible(PlanEditorUtils.getPlanEditor(this), t.getInState()))
				{
					targets.add(t);
				}
			}
					
		}
		
		if ((((State) getModel()).getInPlan()!=null)) {			
			EList<EntryPoint> eps = ((State) getModel()).getInPlan().getEntryPoints();
			EntryPoint matchedEP = null;
			for(EntryPoint ep : eps) {
				if (ep.getState() == getModel()) {
					matchedEP = ep;
					break;
				}
			}
			
			if (matchedEP != null) {
				targets.add(new EntryPointStateDummyConnection(matchedEP, (State) getModel()));
			}
			
			if (targets.isEmpty()) {
				targets = Collections.emptyList();
			}
		}
		return targets;
	}
	
	/**
	 * Convenience method for getting the state model object
	 * @return
	 */
	private State getState(){
		return (State)getModel();
	}
	
	@Override
	protected List getExpandedChildren() {
		List<PlanElement> children = new ArrayList<PlanElement>();
		children.addAll(getState().getPlans());
		for(AbstractPlan p : getState().getPlans())
		{
			if(p.eIsProxy())
			{
				System.out.println(p +" is a Proxy!!!");
			}
		}
		return children;
	}
	
	@Override
	protected void handleModelChanged(Notification n) {
		super.handleModelChanged(n);
		refreshSourceConnections();
		refreshTargetConnections();
	}
	
	public void removeChild(AbstractPlanEditPart child) {
		children.remove(child);
	}
	
	public void addChild(AbstractPlanEditPart child, int index) {
		if (index >= 0)
			children.add(index, child);
		else
			children.add(child);
	}
}
