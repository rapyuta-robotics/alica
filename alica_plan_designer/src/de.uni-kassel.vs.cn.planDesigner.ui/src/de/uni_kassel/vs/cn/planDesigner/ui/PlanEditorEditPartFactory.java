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
package de.uni_kassel.vs.cn.planDesigner.ui;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.EditPartFactory;

import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.FailureState;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;
import de.uni_kassel.vs.cn.planDesigner.alica.PostCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.PreCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.RuntimeCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.SuccessState;
import de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.EntryPointStateDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.SynchedTransitionDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.AbstractPlanStateEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.BehaviourConfigurationEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.ConditionEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.EntryPointEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.EntryPointStateConnectionEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.FailureStateEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.PlanDiagramEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.PlanEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.PlanStateEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.PlanTypeEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.PlanningProblemEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.StateEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.SuccessStateEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.SynchedTransitionEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.SynchronisationEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.TaskEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.TerminalStateEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.TransitionEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class PlanEditorEditPartFactory implements EditPartFactory {

	public EditPart createEditPart(EditPart context, Object model) {
		EditPart child = null;

		if (model instanceof Resource) {
			child = new PlanDiagramEditPart();
		} else if (model instanceof Plan) {
			if (context instanceof StateEditPart) {
				child = new PlanStateEditPart();
			} else {
				child = new PlanEditPart();
			}
		} else if (model instanceof PlanningProblem) {
			child = new PlanningProblemEditPart();
		} else if (model instanceof Transition) {
			child = new TransitionEditPart();
		} else if (model instanceof EntryPoint) {
			child = new EntryPointEditPart();
		} else if (model instanceof EntryPointStateDummyConnection) {
			child = new EntryPointStateConnectionEditPart();
		} else if (model instanceof Synchronisation) {
			child = new SynchronisationEditPart();
		} else if (model instanceof SynchedTransitionDummyConnection) {
			child = new SynchedTransitionEditPart();
		} else if (model instanceof SuccessState) {
			child = new SuccessStateEditPart();
		} else if (model instanceof FailureState) {
			child = new FailureStateEditPart();
		} else if (model instanceof State) {
			child = new StateEditPart();
		} 
		else if (model instanceof PostCondition) {
			if (context instanceof AbstractPlanStateEditPart)
				child = new ConditionEditPart(PlanDesignerConstants.ICON_POST_CONDITION_12);
			else if (context instanceof TerminalStateEditPart)
				child = new ConditionEditPart(PlanDesignerConstants.ICON_POST_CONDITION_12);
				//child = new ExitPointResultEditPart();
			else
				child = new ConditionEditPart(PlanDesignerConstants.ICON_POST_CONDITION_20);
		} else if (model instanceof RuntimeCondition) {
			if (context instanceof AbstractPlanStateEditPart) {
				child = new ConditionEditPart(PlanDesignerConstants.ICON_RUNTIME_CONDITION_12);
			} else {
				child = new ConditionEditPart(PlanDesignerConstants.ICON_RUNTIME_CONDITION_20);
			}
		} else if (model instanceof PreCondition) {
			if (context instanceof AbstractPlanStateEditPart) {
				child = new ConditionEditPart(PlanDesignerConstants.ICON_PRE_CONDITION_12);
			} else {
				child = new ConditionEditPart(PlanDesignerConstants.ICON_PRE_CONDITION_20);
			}
		} else if (model instanceof BehaviourConfiguration) {
			child = new BehaviourConfigurationEditPart();
		} else if (model instanceof PlanType) {
			child = new PlanTypeEditPart();
		} else if (model instanceof Task) {
			child = new TaskEditPart();
		}  
		
		child.setModel(model);
	

		return child;
	}

}
