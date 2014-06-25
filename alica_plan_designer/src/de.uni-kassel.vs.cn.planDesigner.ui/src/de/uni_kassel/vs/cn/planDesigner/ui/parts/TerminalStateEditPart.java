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
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.ImageFigure;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.GraphicalEditPart;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.alica.PostCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.TerminalState;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.IModelExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.PlanElementExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.SynchedTransitionDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PMLFlowLayoutEditPolicy;

public class TerminalStateEditPart extends StateEditPart {
	
	protected ImageFigure pointFigure;
	
	@Override
	protected List getModelChildren() {
		List<EObject> result = new ArrayList<EObject>();
		
		if(getTerminalState().getPostCondition() != null)
			result.add(getTerminalState().getPostCondition());
		
		return result;
		
	}
	
	private IModelExclusionAdapter modelExclusionAdapter;
		
	
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
					ex.add(SynchedTransitionDummyConnection.class.getName());
					ex.add(AlicaPackage.eINSTANCE.getAbstractPlan().getName());
					ex.add(AlicaPackage.eINSTANCE.getPlan().getName());
					ex.add(AlicaPackage.eINSTANCE.getPlanningProblem().getName());
					ex.add(AlicaPackage.eINSTANCE.getPlanType().getName());
					ex.add(AlicaPackage.eINSTANCE.getBehaviourConfiguration().getName());
					return ex;
				}
			};
		}
		return this.modelExclusionAdapter;
	}
	

	protected TerminalState getTerminalState(){
		return (TerminalState)getModel();
	}

	@Override
	protected List<PlanElement> getExpandedChildren() {
		List<PlanElement> children = new ArrayList<PlanElement>();
		PostCondition r = getTerminalState().getPostCondition();
		if(r!=null) {
			children.add(r);
		}
		return children;
	}

	@Override
	protected void addChildVisual(EditPart childEditPart, int index) {
		// We just want to append the child
		IFigure child = ((GraphicalEditPart)childEditPart).getFigure();
		getContentPane().add(child);
	}
	
	@Override
	protected void createEditPolicies() {
		super.createEditPolicies();
	}
}
