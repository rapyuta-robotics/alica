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
package de.uni_kassel.vs.cn.planDesigner.codegeneration.adapter;

import org.eclipse.core.resources.IFile;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.PlanAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;

public class IFilePlanAdapter implements PlanAdapter
{
	private final Object selection;

	public IFilePlanAdapter(Object selection)
	{
		this.selection = selection;
	}

	public Plan getAdaptedPlan()
	{
		Plan adaptedPlan = null;
		if(selection instanceof IFile)
		{
			IFile file = (IFile)selection;
			
			// Load the model
			ResourceSet rSet = CommonUtils.getAlicaResourceSet();
			Resource res = rSet.getResource(org.eclipse.emf.common.util.URI
					.createPlatformResourceURI(file.getFullPath()
							.toString(), true), true);

			adaptedPlan = (Plan) res.getContents().get(0);
		}
		
		return adaptedPlan;
	}

}
