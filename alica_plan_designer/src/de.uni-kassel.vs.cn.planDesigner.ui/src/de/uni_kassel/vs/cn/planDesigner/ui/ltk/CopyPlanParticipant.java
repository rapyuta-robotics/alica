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
import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.ltk.core.refactoring.Change;
import org.eclipse.ltk.core.refactoring.CompositeChange;
import org.eclipse.ltk.core.refactoring.RefactoringStatus;
import org.eclipse.ltk.core.refactoring.participants.CheckConditionsContext;
import org.eclipse.ltk.core.refactoring.participants.CopyParticipant;

import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;

public class CopyPlanParticipant extends CopyParticipant {

	public CopyPlanParticipant() {
		// TODO Auto-generated constructor stub
	}

	@Override
	public RefactoringStatus checkConditions(IProgressMonitor pm,
			CheckConditionsContext context) throws OperationCanceledException {
		return new RefactoringStatus();
	}

	@Override
	public Change createChange(IProgressMonitor pm) throws CoreException,
			OperationCanceledException {

		CompositeChange compChange = new CompositeChange("Generate new IDs");
		
		Object[] elementsToCopy = getProcessor().getElements();
		ResourceSet rSet = CommonUtils.getAlicaResourceSet();
		
		for(Object o : elementsToCopy)
		{
			IFile copiedFile = ((IFile)o);
			Resource resource = rSet.getResource(URI.createPlatformResourceURI(copiedFile.getFullPath().toString(),
				true), true);
			
			TreeIterator<EObject> allContents = resource.getAllContents();
			
			while(allContents.hasNext())
			{
				EObject next = allContents.next();
				if(next instanceof PlanElement)
				{
					compChange.add(new GenerateNewAlicaIDChange((PlanElement)next));
				}
			}
			
		}
		
		
		
		return compChange;
		
		
	}

	@Override
	public String getName() {
		return "Generate new IDs";
	}

	@Override
	protected boolean initialize(Object element) {
		boolean result = false;
		if(element instanceof IFile && ((IFile)element).getFileExtension().equals("pml")){
			result = true;
		}
		return result;
	}

}
