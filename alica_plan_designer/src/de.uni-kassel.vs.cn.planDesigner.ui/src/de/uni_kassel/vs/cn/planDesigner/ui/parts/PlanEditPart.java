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

import org.eclipse.draw2d.IFigure;
import org.eclipse.draw2d.Label;
import org.eclipse.draw2d.LayoutManager;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.GraphicalEditPart;
import org.eclipse.gef.GraphicalViewer;
import org.eclipse.gef.LayerConstants;
import org.eclipse.gef.editparts.AbstractGraphicalEditPart;

import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PostCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.PreCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.RuntimeCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.figures.HiddenElementsInfoFigure;
import de.uni_kassel.vs.cn.planDesigner.ui.figures.PlanFigure;
import de.uni_kassel.vs.cn.planDesigner.ui.layout.InfoLocator;
import de.uni_kassel.vs.cn.planDesigner.ui.layout.PMLXYLayout;
import de.uni_kassel.vs.cn.planDesigner.ui.policies.PlanXYLayoutEditPolicy;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public class PlanEditPart extends AbstractPlanEditPart {

	private PlanFigure planFigure;

	@Override
	protected IFigure createFigure() {
		this.planFigure = new PlanFigure(this);
		this.planFigure.getMainFigure().setLayoutManager(new PMLXYLayout(this));

		return this.planFigure;
	}

	@Override
	protected void addAllAdapters() {
		super.addAllAdapters();

		// / Register the editPart with the ui extensionmodel
		PmlUiExtension extension = CommonUtils.getUIAwareEditorAdapter(this)
				.getUIExtension(getEObjectModel(), true);
		if (extension != null)
			this.adapter.addToObject(extension);

		Plan p = (Plan) getModel();
		this.adapter.addToObject(p);

		for (EntryPoint ep : p.getEntryPoints()) {
			this.adapter.addToObject(ep);
		}
	}

	@Override
	protected void removeChildVisual(EditPart childEditPart) {
		IFigure figureToRemove = findFigure(childEditPart);
		figureToRemove.remove(((AbstractGraphicalEditPart) childEditPart)
				.getFigure());
	}

	@Override
	protected void addChildVisual(EditPart childEditPart, int index) {
		IFigure figureToAdd = findFigure(childEditPart);
		IFigure childFigure = ((AbstractGraphicalEditPart) childEditPart)
				.getFigure();
		// Because we want to place entry and exitpoint in other figures
		// than all other content, we have to make a little check here
		if (!figureToAdd.equals(getMainFigure())){
			figureToAdd.add(childFigure, -1);
		}else{
			figureToAdd.add(childFigure, getFigure().getLayoutManager()
					.getConstraint(childFigure), -1);
		}
	}

	protected IFigure findFigure(EditPart part) {
		if (part instanceof EntryPointEditPart)
			return getPlanFigure().getEntryPointFigure();
		else if (part instanceof SuccessStateEditPart
				|| part instanceof FailureStateEditPart)
			return getPlanFigure().getExitPointFigure();
		else if (part.getModel() instanceof PostCondition)
			return getPlanFigure().getResultHolder();
		else if (part.getModel() instanceof PreCondition)
			return getPlanFigure().getPreconditionHolder();
		else if (part.getModel() instanceof RuntimeCondition)
			return getPlanFigure().getRuntimeConditionHolder();
		else
			return getContentPane();
	}

	@Override
	public IFigure getContentPane() {
		return getPlanFigure().getMainFigure();
	}

	@Override
	protected void createEditPolicies() {
		super.createEditPolicies();
		installEditPolicy(EditPolicy.LAYOUT_ROLE, new PlanXYLayoutEditPolicy());
	}

	@Override
	protected List<Object> getModelChildren() {
		List<Object> children = super.getModelChildren();

		// If the AbstractPlan didn't have any children, we create
		// a new list, because we cannot work with a Collection.EMPTY_LIST
		if (children.size() == 0)
			children = new ArrayList<>();

		for (State s : ((Plan) getModel()).getStates()) {
			// Care for hidden states
			if (CommonUtils.isVisible(PlanEditorUtils.getPlanEditor(this), s)) {
				children.add(s);
			}

		}

		for (EntryPoint ep : getPlan().getEntryPoints())
			children.add(ep);

		for (Synchronisation sync : getPlan().getSynchronisations())
			children.add(sync);

		return children;
	}

	@Override
	protected void refreshVisuals() {
		super.refreshVisuals();
		if (((Plan)getModel()).isMasterPlan())
		{
			getPlanFigure().getNameHolder().setBackgroundColor(PlanDesignerActivator.getDefault().getColorRegistry().get(PlanDesignerConstants.MASTER_PLAN_LABEL_BACKGROUND_COLOR));
		} else {
			getPlanFigure().getNameHolder().setBackgroundColor(PlanDesignerActivator.getDefault().getColorRegistry().get(PlanDesignerConstants.PLAN_LABEL_BACKGROUND_COLOR));
		}
		refreshHiddenElementsInfo();
	}

	private IFigure hiddenElementsInfo;

	public IFigure getHiddenElementsInfo() {
		if (hiddenElementsInfo == null) {
			hiddenElementsInfo = new HiddenElementsInfoFigure(
					(GraphicalViewer) getViewer(), new InfoLocator(
							(GraphicalViewer) getViewer()));
		}
		return hiddenElementsInfo;
	}

	private void refreshHiddenElementsInfo() {
		IFigure layer = getLayer(LayerConstants.FEEDBACK_LAYER);
		if (layer.getChildren().contains(getHiddenElementsInfo())) {
			layer.getChildren().remove(getHiddenElementsInfo());
		}

		if (containsHiddenElements()) {
			layer.add(getHiddenElementsInfo());
		}

		layer.revalidate();
		layer.repaint();
	}

	/**
	 * Checks weather the plan contains hidden elements
	 * 
	 * @return
	 */
	private boolean containsHiddenElements() {
		boolean match = false;

		for (State state : getPlan().getStates()) {
			match = !CommonUtils.isVisible(PlanEditorUtils.getPlanEditor(this),
					state);
			if (match) {
				break;
			}
		}

		return match;
	}

	/**
	 * Convenience method which return the plan
	 * 
	 * @return
	 */
	protected Plan getPlan() {
		return (Plan) getModel();
	}

	@Override
	public Label getNameLabel() {
		return getPlanFigure().getNameLabel();
	}

	@Override
	public IFigure getMainFigure() {
		return getPlanFigure().getMainFigure();
	}

	@Override
	protected void reorderChild(EditPart child, int index) {
		// Save the constraint of the child so that it does not
		// get lost during the remove and re-add.
		IFigure childFigure = ((GraphicalEditPart) child).getFigure();
		LayoutManager layout = findFigure(child).getLayoutManager();
		Object constraint = null;
		if (layout != null)
			constraint = layout.getConstraint(childFigure);

		removeChildVisual(child);
		List children = getChildren();
		children.remove(child);
		children.add(index, child);
		addChildVisual(child, index);

		setLayoutConstraint(child, childFigure, constraint);
	}

	public PlanFigure getPlanFigure() {
		return planFigure;
	}
}
