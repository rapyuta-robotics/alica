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
package de.uni_kassel.vs.cn.planDesigner.ui.policies;

import java.util.Collections;
import java.util.Set;

import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.AddCommand;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.util.TransactionUtil;
import org.eclipse.gef.Request;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.commands.UnexecutableCommand;
import org.eclipse.gef.editpolicies.GraphicalNodeEditPolicy;
import org.eclipse.gef.requests.CreateConnectionRequest;
import org.eclipse.gef.requests.ReconnectRequest;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.PreCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.alica.impl.TransitionImpl;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.IModelExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CommandWrap4EMF;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.EntryPointStateDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.SynchedTransitionDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.EntryPointEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.EntryPointStateConnectionEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.SynchedTransitionEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.TransitionEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

/**
 * The PMLConnectionEditPolicy is responsible for handling changes of the plan
 * modelers connections: Transitions, EntryPoint- State-Connections and
 * Synchronisation-Transition-Connections.
 * 
 * Note: There are DummyModelObjects for the latter two.
 * 
 * @author Stephan Opfer
 * 
 */
public class PMLConnectionEditPolicy extends GraphicalNodeEditPolicy {

	@Override
	public boolean understandsRequest(Request req) {
		super.understandsRequest(req);
		return false;
	};

	@Override
	protected Command getConnectionCompleteCommand(CreateConnectionRequest request) {
		
		CommandWrap4EMF cmd = (CommandWrap4EMF) request.getStartCommand();

		org.eclipse.emf.common.command.Command createCmd = cmd.unwrap();
		CompoundCommand compound = new CompoundCommand(0);

		EObject target = (EObject) request.getTargetEditPart().getModel();
		EObject source = (EObject) request.getSourceEditPart().getModel();

		// We will not allow the source to be the same as the target
		if (target.equals(source))
			return UnexecutableCommand.INSTANCE;

		PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain) TransactionUtil.getEditingDomain(target);

		Object result = createCmd.getResult().toArray(new Object[1])[0];
		if (result instanceof EntryPointStateDummyConnection) {
			if (!(source instanceof EntryPoint && target instanceof State)) {
				return UnexecutableCommand.INSTANCE;
			}
			compound.append(SetCommand.create(editingDomain, source, AlicaPackage.eINSTANCE.getEntryPoint_State(), target));
		} else if (result instanceof SynchedTransitionDummyConnection) {
			if (!(source instanceof Synchronisation && target instanceof Transition)) {
				return UnexecutableCommand.INSTANCE;
			}
			compound.append(CreateChildCommand.create(editingDomain, target, new CommandParameter(target, AlicaPackage.eINSTANCE.getTransition_Synchronisation(), source), Collections.emptyList()));
//			compound.append(SetCommand.create(editingDomain, target, AlicaPackage.eINSTANCE.getTransition_Synchronisation(), source));
		} else if (result instanceof Transition) {
			if (!(source instanceof State && target instanceof State)) {
				return UnexecutableCommand.INSTANCE;
			}
			//Maybe these are wrong CompoundCommands because with this it didn't work!
			compound.append(createCmd);
			compound.append(SetCommand.create(editingDomain, result, AlicaPackage.eINSTANCE.getTransition_OutState(), target));
			compound.append(SetCommand.create(editingDomain, result, AlicaPackage.eINSTANCE.getTransition_InState(), source));
			
			//Makes the transition enabled with the command for that!
			PreCondition condition = ((Transition) result).getPreCondition();
			condition = (PreCondition) AlicaFactory.eINSTANCE.create(AlicaPackage.eINSTANCE.getPreCondition());
			condition.setEnabled(true);
			compound.append(SetCommand.create(editingDomain, result, AlicaPackage.eINSTANCE.getTransition_PreCondition(), condition));
//			compound.append(CreateChildCommand.create(editingDomain, result, new CommandParameter(result, AlicaPackage.eINSTANCE.getTransition_OutState(), target), Collections.emptyList()));
//			compound.append(CreateChildCommand.create(editingDomain, result, new CommandParameter(result, AlicaPackage.eINSTANCE.getTransition_InState(), source), Collections.emptyList()));
		}

		return new CommandWrap4EMF(compound);
	}

	@Override
	protected Command getConnectionCreateCommand(CreateConnectionRequest request) {
		Command createConnectionCmd = UnexecutableCommand.INSTANCE;

		IModelExclusionAdapter exclusionAdapter = (IModelExclusionAdapter) getHost().getAdapter(IModelExclusionAdapter.class);
		Set<String> exclusionSet = null;
		if (exclusionAdapter != null)
			exclusionSet = exclusionAdapter.getExclusionClasses();

		// Only create the command if the child isn't in the exclusion set
		if (exclusionSet == null || (exclusionSet != null && !exclusionSet.contains(request.getNewObject().getClass().getName()))) {

			EObject source = (EObject) getHost().getModel();
			PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain) TransactionUtil.getEditingDomain(source);
			PlanEditor editor = PlanEditorUtils.getPlanEditor(getHost());

			if (request.getNewObject() instanceof EntryPointStateDummyConnection) {
				// Initial State Connection
				org.eclipse.emf.common.command.Command createCmd = CreateChildCommand.create(editingDomain, editor.getPlan(), new CommandParameter(null, null,
						request.getNewObject()), Collections.EMPTY_LIST); // unexecutable,
																			// so
																			// just
																			// for
																			// information
																			// transport
				createConnectionCmd = new CommandWrap4EMF(createCmd);
				request.setStartCommand(createConnectionCmd);
			} else if (request.getNewObject() instanceof SynchedTransitionDummyConnection) {
				// Synched Transition Connection
				org.eclipse.emf.common.command.Command createCmd = CreateChildCommand.create(editingDomain, editor.getPlan(), new CommandParameter(null, null,
						request.getNewObject()), Collections.EMPTY_LIST); // unexecutable,
																			// so
																			// just
																			// for
																			// information
																			// transport
				createConnectionCmd = new CommandWrap4EMF(createCmd);
				request.setStartCommand(createConnectionCmd);
			} else if (request.getNewObject() instanceof TransitionImpl) {
				// Transition
				org.eclipse.emf.common.command.Command createCmd = CreateChildCommand.create(editingDomain, editor.getPlan(), new CommandParameter(editor.getPlan(), AlicaPackage.eINSTANCE.getPlan_Transitions(),
						request.getNewObject()), Collections.EMPTY_LIST);
				createConnectionCmd = new CommandWrap4EMF(createCmd);
				request.setStartCommand(createConnectionCmd);
			}
		}

		return createConnectionCmd;
	}

	@Override
	protected Command getReconnectSourceCommand(ReconnectRequest request) {
		Command cmd = UnexecutableCommand.INSTANCE;

		IModelExclusionAdapter exclusionAdapter = (IModelExclusionAdapter) getHost().getAdapter(IModelExclusionAdapter.class);
		Set<String> exclusionSet = null;
		if (exclusionAdapter != null)
			exclusionSet = exclusionAdapter.getExclusionClasses();

		EObject newSource = (EObject) request.getTarget().getModel();
		PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain) TransactionUtil.getEditingDomain(newSource);

		// Only create the command if the child isn't in the exclusion set
		if (exclusionSet == null || (exclusionSet != null && !exclusionSet.contains(newSource.getClass().getName()))) {

			if (request.getConnectionEditPart() instanceof EntryPointStateConnectionEditPart) {
				// Initial State Connection
				cmd = new CommandWrap4EMF(SetCommand.create(editingDomain, newSource, AlicaPackage.eINSTANCE.getEntryPoint_State(), request
						.getConnectionEditPart().getTarget().getModel()));
			} else if (request.getConnectionEditPart() instanceof SynchedTransitionEditPart) {
				// Synched Transition Connection
				cmd = new CommandWrap4EMF(AddCommand.create(editingDomain, newSource, AlicaPackage.eINSTANCE.getSynchronisation_SynchedTransitions(), request
						.getConnectionEditPart().getTarget().getModel()));
			} else if (request.getConnectionEditPart() instanceof TransitionEditPart) {
				// Transition
				cmd = new CommandWrap4EMF(SetCommand.create(editingDomain, request.getConnectionEditPart().getModel(),
						AlicaPackage.eINSTANCE.getTransition_InState(), newSource));
			}
		}

		return cmd;
	}

	@Override
	protected Command getReconnectTargetCommand(final ReconnectRequest request) {

		Command cmd = UnexecutableCommand.INSTANCE;
		IModelExclusionAdapter exclusionAdapter = (IModelExclusionAdapter) getHost().getAdapter(IModelExclusionAdapter.class);
		Set<String> exclusionSet = null;
		if (exclusionAdapter != null)
			exclusionSet = exclusionAdapter.getExclusionClasses();

		EObject newTarget = (EObject) request.getTarget().getModel();
		PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain) TransactionUtil.getEditingDomain(newTarget);

		// Only create the command if the child isn't in the exclusion set
		if (exclusionSet == null || (exclusionSet != null && !exclusionSet.contains(newTarget.getClass().getName()))) {

			if (request.getConnectionEditPart() instanceof EntryPointStateConnectionEditPart) {
				// Initial State Connection
				cmd = new CommandWrap4EMF(SetCommand.create(editingDomain, newTarget, AlicaPackage.eINSTANCE.getState_EntryPoint(), request
						.getConnectionEditPart().getSource().getModel()));
			} else if (request.getConnectionEditPart() instanceof SynchedTransitionEditPart) {
				// Synched Transition Connection
				CompoundCommand cc = new CompoundCommand("Change target of synched transition connection");
				Synchronisation sync = (Synchronisation) request.getConnectionEditPart().getSource().getModel();
				Transition oldTrans = (Transition) request.getConnectionEditPart().getTarget().getModel();
				cc.append(SetCommand.create(editingDomain, oldTrans, AlicaPackage.eINSTANCE.getTransition_Synchronisation(), null));
				cc.append(SetCommand.create(editingDomain, newTarget, AlicaPackage.eINSTANCE.getTransition_Synchronisation(), sync));
				cmd = new CommandWrap4EMF(cc);
			} else if (request.getConnectionEditPart() instanceof TransitionEditPart) {
				// Transition
				cmd = new CommandWrap4EMF(SetCommand.create(editingDomain, request.getConnectionEditPart().getModel(),
						AlicaPackage.eINSTANCE.getTransition_OutState(), newTarget));
			}
		}

		return cmd;
	}

}
