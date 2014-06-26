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
package de.uni_kassel.vs.cn.planDesigner.ui.util;

import java.io.IOException;

import org.eclipse.core.resources.IFile;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.transaction.TransactionalEditingDomain;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;

public class PlanMapper {

	private final IFile file;
	
	private PMLTransactionalEditingDomain editingDomain;
	
	private Resource planResource;
	
	private Plan loadedPlan;

	public PlanMapper(IFile file) {
		this.file = file;
	}
	
	public void save() throws IOException{
		Resource planResource = getPlanResource();
		
		try {
			// Copy the plan which will be serialized
			Plan copiedPlan = (Plan)EcoreUtil.copy(getPlan());
			
			// Replace the contents of the planResource
			// with the copiedPlan...
			planResource.getContents().clear();
			planResource.getContents().add(copiedPlan);
			
			// ...save...
			planResource.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
			
			// ...and put the original plan back
			planResource.getContents().clear();
			planResource.getContents().add(getPlan());
		} catch (Exception e) {
			e.printStackTrace();
		}
		
	}
	
	public Resource load(){
		// Load the file with the editingDomain
		Resource resource = getEditingDomain().load(file);
		setPlanResource(resource);
		
		return planResource;
	}

	private PMLTransactionalEditingDomain getEditingDomain() {
		if(editingDomain == null){
			editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		}
		
		return editingDomain;
		
	}
	
	public Plan getPlan(){
		if(loadedPlan == null){
			loadedPlan = (Plan)getPlanResource().getContents().get(0);
		}
		
		return loadedPlan;
	}
	
	
	public Resource getPlanResource() {
		return planResource;
	}

	protected void setPlanResource(Resource planResource) {
		this.planResource = planResource;
	}
}
