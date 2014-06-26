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
package de.uni_kassel.vs.cn.planDesigner.ui.ltk;

import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.OperationCanceledException;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.edit.command.RemoveCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.ltk.core.refactoring.Change;
import org.eclipse.ltk.core.refactoring.RefactoringStatus;

import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;

public class DeletePlanReferenceChange extends Change {
	
	private PMLTransactionalEditingDomain domain;
	
	private EStructuralFeature feature;
	
	private PlanElement value;
	
	private EObject owner;
	
	private Command removeCommand;
	
	public DeletePlanReferenceChange(PMLTransactionalEditingDomain domain, EObject owner, EStructuralFeature feature, PlanElement value) {
		this.domain = domain;
		this.owner = owner;
		this.feature = feature;
		this.value = value;
	}

	@Override
	public Object getModifiedElement() {
		return owner;
	}

	@Override
	public String getName() {
		return "Removing plan " +value.getName() +" from " +owner +" in " +owner.eResource().getURI();
	}

	@Override
	public void initializeValidationData(IProgressMonitor pm) {
		if(feature.isMany())
			removeCommand = RemoveCommand.create(domain, owner, feature, value);
		else
			removeCommand = SetCommand.create(domain, owner, feature, null);
	}

	@Override
	public RefactoringStatus isValid(IProgressMonitor pm) throws CoreException,
			OperationCanceledException {
		RefactoringStatus status = new RefactoringStatus();
		
		if(!removeCommand.canExecute())
			status.addFatalError("Cannot delete " +value +" from " +owner);
		
		return status;
	}

	@Override
	public Change perform(IProgressMonitor pm) throws CoreException {
		try {
			domain.getCommandStack().execute(removeCommand);
			
			// Save the resource in which we have removed something
			Resource res = owner.eResource();
			res.save(res.getResourceSet().getLoadOptions());
			// reload the resource
			res.unload();
			res.load(res.getResourceSet().getLoadOptions());
			
		} catch (Exception e) {
			Status s = new Status(Status.ERROR, PlanDesignerActivator.PLUGIN_ID, 94, "Remove command cannot be executed", e);
			throw new CoreException(s);
		}
		
		return null;
	}

}
