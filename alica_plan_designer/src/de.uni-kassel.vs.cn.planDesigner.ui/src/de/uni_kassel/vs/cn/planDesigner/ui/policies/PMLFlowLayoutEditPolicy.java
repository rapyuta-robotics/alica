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

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.AddCommand;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.transaction.util.TransactionUtil;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.Request;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.commands.UnexecutableCommand;
import org.eclipse.gef.editpolicies.FlowLayoutEditPolicy;
import org.eclipse.gef.requests.CreateRequest;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.PostCondition;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.IModelExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CommandWrap4EMF;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.RemoveReadOnlyCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;

public class PMLFlowLayoutEditPolicy extends FlowLayoutEditPolicy {

	@Override
	protected Command createAddCommand(EditPart child, EditPart after) {
		Command cmd = UnexecutableCommand.INSTANCE;
		
		// We only know how to build children in EObject containers
		if(getHost().getModel() instanceof EObject){
			final EObject parent = (EObject)getHost().getModel();
			
			IModelExclusionAdapter exclusionAdapter = (IModelExclusionAdapter)getHost().getAdapter(IModelExclusionAdapter.class);
			Set<String> exclusionSet = null;
			if(exclusionAdapter != null)
				exclusionSet = exclusionAdapter.getExclusionClasses();

			EObject eObj = (EObject) child.getModel();
			System.out.println(eObj);
			// Only create the command if the child isn't in the exclusion set
			if(exclusionSet == null || (exclusionSet != null && !exclusionSet.contains(eObj.eClass().getName()))){
				final PMLTransactionalEditingDomain editingDomain =  (PMLTransactionalEditingDomain)TransactionUtil.getEditingDomain(parent);
				
				editingDomain.getCommandStack().execute(new RemoveReadOnlyCommand(editingDomain));
				
				org.eclipse.emf.common.command.Command createChildCommand = org.eclipse.emf.common.command.UnexecutableCommand.INSTANCE;
			
				if(eObj instanceof AbstractPlan){
					createChildCommand = AddCommand.create(
							editingDomain, 
							parent, 
							AlicaPackage.eINSTANCE.getState_Plans(), 
							eObj);
				} else if (eObj instanceof PostCondition) {
					createChildCommand = AddCommand.create(
							editingDomain, 
							parent, 
							AlicaPackage.eINSTANCE.getTerminalState_PostCondition(), 
							eObj);
				}
				
				cmd = new CommandWrap4EMF(createChildCommand);
			}
		}
		
		return cmd;
	}

	@Override
	protected Command createMoveChildCommand(EditPart child, EditPart after) {
		// TODO Auto-generated method stub
		return null;
	}
	
	@Override
	protected Command getMoveChildrenCommand(Request request){
		return null;
	}
	@Override
	protected Command getCreateCommand(CreateRequest req) {
		return getAddChildCommand(req);
	}
	
	private Command getAddChildCommand(CreateRequest req){
		// TODO: Remove this - only for debugging purposes. This should not be happen!
//		if(req.getNewObject() == null){
//			System.err.println("Creation Factory doesn't return a new Object!");
//			return null;
//		}
		Command cmd = UnexecutableCommand.INSTANCE;
		EObject newChild = (EObject)req.getNewObject();
	
		// We only know how to build children in EObject containers
		if(getHost().getModel() instanceof EObject){
			EObject parent = (EObject)getHost().getModel();
			
			IModelExclusionAdapter exclusionAdapter = (IModelExclusionAdapter)getHost().getAdapter(IModelExclusionAdapter.class);
			Set<String> exclusionSet = null;
			if(exclusionAdapter != null)
				exclusionSet = exclusionAdapter.getExclusionClasses();
			//System.out.println("Add child "+newChild.eClass()+" to "+);			
			// Only create the command if the child isn't in the exclusion set
			if(exclusionSet == null || (exclusionSet != null && !exclusionSet.contains(newChild.eClass().getName()))){
				PMLTransactionalEditingDomain editingDomain =  (PMLTransactionalEditingDomain)TransactionUtil.getEditingDomain(parent);
				
				if (newChild instanceof AbstractPlan) {
					cmd = new CommandWrap4EMF(CreateChildCommand.create(
							editingDomain, 
							parent, 
							new CommandParameter(parent,AlicaPackage.eINSTANCE.getState_Plans(),newChild), 
							Collections.emptyList()));
				} else if (newChild instanceof PostCondition) {
					cmd = new CommandWrap4EMF(CreateChildCommand.create(
							editingDomain, 
							parent, 
							new CommandParameter(parent,AlicaPackage.eINSTANCE.getTerminalState_PostCondition(),newChild), 
							Collections.emptyList()));
				}
			}
		}
		
		return cmd;
	}

}
