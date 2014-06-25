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
package de.uni_kassel.vs.cn.planDesigner.ui.tool;

import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.tools.CreationTool;
import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.wizard.WizardDialog;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLNewPlanWizard;

public class PlanCreationTool extends CreationTool {
	
	@Override
	protected void performCreation(int button) {
		if(getCurrentCommand() == null || !getCurrentCommand().canExecute())
			return;
		
		EditPartViewer viewer = getCurrentViewer();
		
		Plan createdPlan = (Plan)getCreateRequest().getNewObject();
		
		// Create a NewPlan wizard and initialize it with the created plan
		PMLNewPlanWizard wiz = new PMLNewPlanWizard(createdPlan);
		
		WizardDialog dialog = new WizardDialog(viewer.getControl().getShell(), wiz);
		
		dialog.setBlockOnOpen(true);
		if(dialog.open() == Dialog.OK) {
			// Select the new plan
			executeCurrentCommand();
		} else {
			// Undo the command
//			getCurrentViewer().getEditDomain().getCommandStack().undo();
		}
		
	}
}
