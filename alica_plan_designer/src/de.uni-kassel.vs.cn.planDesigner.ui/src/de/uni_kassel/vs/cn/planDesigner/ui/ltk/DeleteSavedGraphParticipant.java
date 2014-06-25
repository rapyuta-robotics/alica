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
import org.eclipse.core.resources.mapping.IResourceChangeDescriptionFactory;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IPath;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.OperationCanceledException;
import org.eclipse.core.runtime.Path;
import org.eclipse.ltk.core.refactoring.Change;
import org.eclipse.ltk.core.refactoring.RefactoringStatus;
import org.eclipse.ltk.core.refactoring.participants.CheckConditionsContext;
import org.eclipse.ltk.core.refactoring.participants.DeleteParticipant;
import org.eclipse.ltk.core.refactoring.participants.ResourceChangeChecker;
import org.eclipse.ltk.core.refactoring.resource.DeleteResourceChange;

public class DeleteSavedGraphParticipant extends DeleteParticipant {
	
	private IFile toDeleteFile;
	
	private IFile taskGraphFile;

	public DeleteSavedGraphParticipant() {
	}

	@Override
	public RefactoringStatus checkConditions(IProgressMonitor pm,
			CheckConditionsContext context) throws OperationCanceledException {
		
		if(getSavedTaskGraphFile().exists()){
			ResourceChangeChecker checker = (ResourceChangeChecker) context.getChecker(ResourceChangeChecker.class);
			IResourceChangeDescriptionFactory deltaFactory= checker.getDeltaFactory();
			
			deltaFactory.delete(getSavedTaskGraphFile());
		}
		
		return new RefactoringStatus();
	}

	@Override
	public Change createChange(IProgressMonitor pm) throws CoreException,
			OperationCanceledException {
		
		Object[] elementsToDelete = getProcessor().getElements();
		
		boolean applicable = true;
		
		for(int i=0; i < elementsToDelete.length; i++)
			if(elementsToDelete[i].equals(getSavedTaskGraphFile())){
				// We want to modifie a resource which the user wants to delete
				// so it doesn't make sense to add a c
				applicable = false;
				break;
			}
		
		if(applicable)
			return new DeleteResourceChange(getSavedTaskGraphFile().getFullPath(), true);
		else 
			return null;
	}
	

	@Override
	public String getName() {
		return "Delete saved Taskgraph";
	}

	@Override
	protected boolean initialize(Object element) {
		boolean result = false;
		if(element instanceof IFile){
			toDeleteFile = (IFile)element;
			if(toDeleteFile.getFileExtension().equals("rset"))
				result = getSavedTaskGraphFile().exists();
		}
		return result;
	}

	private IFile getSavedTaskGraphFile() {
		if(taskGraphFile == null){
			IPath uiExt = new Path(toDeleteFile.getName().substring(0,
					toDeleteFile.getName().lastIndexOf(".")).concat(".graph"));
			
			taskGraphFile = toDeleteFile.getParent().getFile(uiExt);
		}
		return taskGraphFile;
	}

}
