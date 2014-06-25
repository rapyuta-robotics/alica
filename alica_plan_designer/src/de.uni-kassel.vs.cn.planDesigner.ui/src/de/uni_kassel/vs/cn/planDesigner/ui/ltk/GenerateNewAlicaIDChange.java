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
import org.eclipse.ltk.core.refactoring.Change;
import org.eclipse.ltk.core.refactoring.RefactoringStatus;

import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;

public class GenerateNewAlicaIDChange extends Change {
	
	private final PlanElement element;

	public GenerateNewAlicaIDChange(PlanElement element)
	{
		this.element = element;
	}

	@Override
	public Object getModifiedElement() 
	{
		return element;
	}

	@Override
	public String getName() 
	{
		return "ID change of element " +element;
	}

	@Override
	public void initializeValidationData(IProgressMonitor pm) 
	{
		
	}

	@Override
	public RefactoringStatus isValid(IProgressMonitor pm) throws CoreException,
			OperationCanceledException {
		return new RefactoringStatus();
	}

	@Override
	public Change perform(IProgressMonitor pm) throws CoreException {
//		element.setId(element.generateID());
		System.out.println("Setting ID of element " +element +"from <" +element.getId() +"> to <" +element.generateID() +">");
		return null;
	}

}
