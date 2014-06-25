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
import org.eclipse.core.runtime.IPath;
import org.eclipse.ltk.core.refactoring.Refactoring;
import org.eclipse.ltk.core.refactoring.RefactoringDescriptor;
import org.eclipse.ltk.core.refactoring.RefactoringStatus;

public class CopyRefactoringDescriptor extends RefactoringDescriptor
{
	public static final String ID= "de.uni_kassel.vs.cn.planDesigner.ui.ltk.copy.resources"; //$NON-NLS-1$
	
	private IPath[] fResourcePaths;

	protected CopyRefactoringDescriptor()
	{
		super(ID, null, "copy refactoring", "no comment", RefactoringDescriptor.STRUCTURAL_CHANGE);
	}
	
	public void setResourcePaths(IPath[] resourcePath) {
		if (resourcePath == null)
			throw new IllegalArgumentException();
		fResourcePaths= resourcePath;
	}
	
	public IPath[] getResourcePaths() {
		return fResourcePaths;
	}

	@Override
	public Refactoring createRefactoring(RefactoringStatus status)
			throws CoreException
	{
		// TODO Auto-generated method stub
		return null;
	}

}
