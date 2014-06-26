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

import org.eclipse.core.resources.IFile;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.OperationCanceledException;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.ltk.core.refactoring.Change;
import org.eclipse.ltk.core.refactoring.RefactoringStatus;
import org.eclipse.ltk.core.refactoring.participants.CheckConditionsContext;
import org.eclipse.ltk.core.refactoring.participants.DeleteParticipant;

import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class UnloadResourceParticipant extends DeleteParticipant {
	
	private IFile toDeleteFile;
	
	

	private PMLTransactionalEditingDomain editingDomain;

	public UnloadResourceParticipant() {
	}

	@Override
	public RefactoringStatus checkConditions(IProgressMonitor pm,
			CheckConditionsContext context) throws OperationCanceledException {
		return new RefactoringStatus();
	}

	@Override
	public Change createChange(IProgressMonitor pm) throws CoreException,
			OperationCanceledException {
		
		return new UnloadResourceChange(getEditingDomain(), getResourceToFile(toDeleteFile));
	}
	

	@Override
	public String getName() {
		return "Unloading resource";
	}

	@Override
	protected boolean initialize(Object element) {
		boolean result = false;
		if(element instanceof IFile){
			toDeleteFile = (IFile)element;		
			Resource resourceToUnload = getResourceToFile(toDeleteFile);
			result = resourceToUnload != null && resourceToUnload.isLoaded();
		}
		return result;
	}
	
	private Resource getResourceToFile(IFile file){
		return getEditingDomain().getResourceSet().getResource(URI.createPlatformResourceURI(
				file.getFullPath().toString(),true), false);
	}
	
	private PMLTransactionalEditingDomain getEditingDomain(){
		if(editingDomain == null){
			// Establish a connecton to the editingDomain
			editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		}
		return editingDomain;
	}

}
