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

import java.io.IOException;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import org.eclipse.core.resources.IFile;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.emf.workspace.impl.WorkspaceCommandStackImpl;
import org.eclipse.jface.action.IAction;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.ui.IObjectActionDelegate;
import org.eclipse.ui.IWorkbenchPart;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CalculatePlanCardinalitiesCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.EnsurePlanParametrisationConsistencyCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class NormalizePlanAction implements IObjectActionDelegate {
	
	private ISelection selection;
	
	private IWorkbenchPart targetPart;

	public void setActivePart(IAction action, IWorkbenchPart targetPart) {
		this.targetPart = targetPart;
	}

	public void run(IAction action) {
		Iterator iter = ((IStructuredSelection)selection).iterator();
		
		// Get the domain
		PMLTransactionalEditingDomain domain =  (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
				PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		
		Map<IFile, Integer> plan2ModificationNumber = new HashMap<IFile, Integer>();
		
		while(iter.hasNext()){
			IFile file = (IFile)iter.next();
			
			// Load the plan through the domain
			Resource res = domain.load(file);
			Plan p = (Plan)res.getContents().get(0);
			
			CompoundCommand cmp = new CompoundCommand();
			// Now perform the normalization steps. First add missing tasks
			int taskCounter = 0;
			for(EntryPoint ep : p.getEntryPoints()){
				if(ep.getTask() == null){
					cmp.append(
							CreateChildCommand.create(
									domain, 
									ep, 
									new CommandParameter(null,null,AlicaFactory.eINSTANCE.createTask()), 
									Collections.EMPTY_LIST));
					taskCounter++;
				}
			}
			
			// Tell the command to calculate his cardinalities
			cmp.append(new CalculatePlanCardinalitiesCommand(p));
			cmp.append(new EnsurePlanParametrisationConsistencyCommand(p));
			
			((WorkspaceCommandStackImpl)domain.getCommandStack()).execute(cmp.unwrap());
			
			// Save the resource
			try {
				res.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			
			plan2ModificationNumber.put(file, taskCounter);
		}
		
		// Compile the message
		StringBuffer buf = new StringBuffer();
		for(IFile file : plan2ModificationNumber.keySet()){
			buf.append(file.getName() +": " +plan2ModificationNumber.get(file) +" Tasks added.\n");
		}
		
		MessageDialog.openInformation(targetPart.getSite().getShell(), 
				"Normalize Plan(s)", 
				buf.toString());

	}

	public void selectionChanged(IAction action, ISelection selection) {
		this.selection = selection;
	}

}
