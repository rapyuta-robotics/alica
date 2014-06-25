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
import org.eclipse.emf.ecore.EObject;
import org.eclipse.gef.EditPolicy;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.IModelExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.PlanElementExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.EntryPointStateDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.SynchedTransitionDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.DragDropEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.DragDropWithoutOrphan;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.EntryPointLayoutEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PMLConnectionEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PMLFlowLayoutEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class EntryPointEditPart extends PlanElementEditPart {

	protected ImageFigure pointFigure;
	
	private IModelExclusionAdapter modelExclusionAdapter;

	@Override
	protected void addAllAdapters() {
		super.addAllAdapters();

		// Register the editPart with the ui extensionmodel
		if(this.getViewer() != null){
			PmlUiExtension extension = CommonUtils.getUIAwareEditorAdapter(this).getUIExtension(getEObjectModel(), true);
			if (extension != null){
				this.adapter.addToObject(extension);
			}
		}
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
					ex.add(AlicaPackage.eINSTANCE.getPlan().getName());
					ex.add(AlicaPackage.eINSTANCE.getBehaviour().getName());
					ex.add(AlicaPackage.eINSTANCE.getBehaviour_Configurations().getName());
					ex.add(AlicaPackage.eINSTANCE.getCondition().getName());
					ex.add(AlicaPackage.eINSTANCE.getTransition().getName());
					ex.add(SynchedTransitionDummyConnection.class.getName());
					return ex;
				}
			};
		}
		return this.modelExclusionAdapter;
	}

	@Override
	protected IFigure createFigure() {
		Figure holder = new Figure();
		holder.setLayoutManager(new FlowLayout());

		holder.add(getMainFigure());
		return holder;
	}

	@Override
	public IFigure getMainFigure() {
		if (pointFigure == null) {
			pointFigure = new ImageFigure(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_ENTRY_POINT_15));
		}
		return pointFigure;
	}

	@Override
	protected List<?> getModelChildren() {
		List<Object> result = new ArrayList<Object>();

		if (getEntryPoint().getTask() != null)
			result.add(getEntryPoint().getTask());

//		if (getEntryPoint().getState() != null) {
//			result.add( new EntryPointStateDummyConnection(getEntryPoint(), getEntryPoint().getState()) );
//		} 
//		
		return result;
	}

	protected EntryPoint getEntryPoint() {
		return (EntryPoint) getModel();
	}

	@Override
	protected void createEditPolicies() {
		super.createEditPolicies();
		installEditPolicy(EditPolicy.LAYOUT_ROLE, new EntryPointLayoutEditPolicy());
		installEditPolicy(EditPolicy.GRAPHICAL_NODE_ROLE, new PMLConnectionEditPolicy());
		installEditPolicy("DragDropRole", new DragDropWithoutOrphan());
	}

	@Override
	protected List<EntryPointStateDummyConnection> getModelSourceConnections() {
		List<EntryPointStateDummyConnection> sources = null;
		if (getEntryPoint().getState() != null) {
			sources = new ArrayList<EntryPointStateDummyConnection>();
			sources.add( new EntryPointStateDummyConnection(getEntryPoint(), getEntryPoint().getState()) );
		} else {
			sources = Collections.emptyList();
		}
		return sources;
	}
}
