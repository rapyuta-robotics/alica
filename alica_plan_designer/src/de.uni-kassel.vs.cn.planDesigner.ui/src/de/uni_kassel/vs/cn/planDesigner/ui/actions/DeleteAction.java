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
package de.uni_kassel.vs.cn.planDesigner.ui.actions;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.edit.command.DeleteCommand;
import org.eclipse.emf.edit.command.RemoveCommand;
import org.eclipse.gef.EditPart;
import org.eclipse.jface.viewers.IStructuredSelection;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.PlanEditPart;

public class DeleteAction extends org.eclipse.emf.edit.ui.action.DeleteAction {

	private Map<Object, EditPart> objectToEditPart = new HashMap<Object, EditPart>();

	// Overridden to extract the eobjects from selected editparts
	@Override
	public boolean updateSelection(IStructuredSelection selection) {
		List<?> list = selection.toList();
		command = createCommand(calculateSelection(list));
		return command.canExecute();
	}

	private Set<Object> calculateSelection(List<?> selection) {
		// Clear the map
		objectToEditPart.clear();

		Set<Object> deleteSelection = new HashSet<Object>();

		for (Object o : selection) {
			if (o instanceof EditPart && !(o instanceof PlanEditPart)) {
				Object model = ((EditPart) o).getModel();
				objectToEditPart.put(model, (EditPart) o);

				// Tasks are not deleted by this action, cause there is a task
				// repository...
				if (model instanceof Task)
					continue;

				deleteSelection.add(model);

				// Also delete the transitions attached to a state
				if (model instanceof State) {
					State s = (State) model;
					for (Transition t : s.getOutTransitions()) {
						deleteSelection.add(t);
					}
					for (Transition t : s.getInTransitions()) {
						deleteSelection.add(t);
					}
				}
				if(model instanceof Synchronisation){
					Synchronisation s = (Synchronisation) model;
					for(Transition t : s.getSynchedTransitions()){
						deleteSelection.add(t);
					}
					
				}

			}
		}
		return deleteSelection;
	}

	public Command createCommand(Collection<?> selection) {
		/*
		 * Divide the selection into two: Because we have AbstractPlans added to
		 * states, we cannot use the DeleteCommand, since it would delete all
		 * occurrences of an AbstractPlan in all states. We also cannot use a
		 * RemoveCommand because the AbstractPlan is not a containment feature
		 * of a state.
		 */

		CompoundCommand cmd = new CompoundCommand();
		Collection<Object> speciallyHandled = new ArrayList<Object>();
		for (Object o : selection) {
			if (o instanceof AbstractPlan) {
				// Manually create a RemoveCommand
				speciallyHandled.add(o);
				cmd.append(RemoveCommand.create(domain, objectToEditPart.get(o).getParent().getModel(), AlicaPackage.eINSTANCE.getState_Plans(), o));
			} 
//			else if (o instanceof EntryPointStateDummyConnection) {
//				cmd.append(SetCommand.create(domain, objectToEditPart.get(o).getParent().getModel(), AlicaPackage.eINSTANCE.getEntryPoint_State(), null));
//			} else if (o instanceof SynchedTransitionDummyConnection) {
//				SynchedTransitionDummyConnection stdc = (SynchedTransitionDummyConnection) o;
//				cmd.append(RemoveCommand.create(domain, objectToEditPart.get(o).getParent().getModel(), AlicaPackage.eINSTANCE.getSynchronisation_SynchedTransitions(), stdc.getTarget()));
//			}
		}
		selection.removeAll(speciallyHandled);

		if (!selection.isEmpty())
			cmd.append(DeleteCommand.create(domain, selection));

		return cmd;
	}

}
