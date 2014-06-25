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

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Set;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.mapping.IResourceChangeDescriptionFactory;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.OperationCanceledException;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.emf.ecore.EStructuralFeature.Setting;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.emf.workspace.util.WorkspaceSynchronizer;
import org.eclipse.ltk.core.refactoring.Change;
import org.eclipse.ltk.core.refactoring.CompositeChange;
import org.eclipse.ltk.core.refactoring.RefactoringStatus;
import org.eclipse.ltk.core.refactoring.participants.CheckConditionsContext;
import org.eclipse.ltk.core.refactoring.participants.DeleteParticipant;
import org.eclipse.ltk.core.refactoring.participants.ResourceChangeChecker;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public class DeletePlanReferenceParticipant extends DeleteParticipant {
	
	private IFile toDeleteFile;
	
	private Collection<Setting> usages;
	
	private PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
			PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
	
	private Plan planToDelete;

	public DeletePlanReferenceParticipant() {
	}

	@Override
	public RefactoringStatus checkConditions(IProgressMonitor pm,
			CheckConditionsContext context) throws OperationCanceledException {
		RefactoringStatus result = new RefactoringStatus();
		
		// We will add each resource which we modify to the deltaFactory.
		// We add those resources in which we will delete references to planToDelete.
		Collection<Setting> usages = getUsages();
		
		if(!usages.isEmpty()){
			ResourceChangeChecker checker = (ResourceChangeChecker) context.getChecker(ResourceChangeChecker.class);
			IResourceChangeDescriptionFactory deltaFactory= checker.getDeltaFactory();
			
			for(Setting setting : usages){
				EObject owner = setting.getEObject();
				
				Resource ownerResource = owner.eResource();
				IFile fileToChange = WorkspaceSynchronizer.getUnderlyingFile(ownerResource);
				deltaFactory.change(fileToChange);
				
			}
			result.addWarning("There are plans which reference the element you want to delete. " +
					"If you continue those references will be removed.");
		}
		
		return result;

	}

	@Override
	public Change createChange(IProgressMonitor pm) throws CoreException,
			OperationCanceledException {
		CompositeChange cmpChange = null;
		
		Collection<Setting> usages = getUsages();
		
		if(!usages.isEmpty()){
			List<Change> changes = new ArrayList<Change>();
			
			for(Setting setting : usages)
				addUsageIfApplicable(changes, setting);
			
			if(!changes.isEmpty()){
				cmpChange = new CompositeChange("Delete plan references");
				cmpChange.addAll(changes.toArray(new Change[changes.size()]));
			}
				
		}
		
		return cmpChange;
	}
	
	/**
	 * Adds a DeletePlanReferenceChange if the Re 
	 * @param cmpChange
	 * @param setting
	 */
	private void addUsageIfApplicable(List<Change> changes, Setting setting){
		EStructuralFeature feature = setting.getEStructuralFeature();
		EObject owner = setting.getEObject();
		
		Resource ownerResource = owner.eResource();
		IFile fileToChange = WorkspaceSynchronizer.getUnderlyingFile(ownerResource);
		
		Object[] elementsToDelete = getProcessor().getElements();
		
		boolean applicable = true;
		
		for(int i=0; i < elementsToDelete.length; i++)
			if(elementsToDelete[i].equals(fileToChange)){
				// We want to modifie a resource which the user wants to delete
				// so it doesn't make sense to add a c
				applicable = false;
				break;
			}
				
		
		if(applicable)
			changes.add(new DeletePlanReferenceChange(editingDomain, owner, feature, getPlanToDelete()));
	}

	@Override
	public String getName() {
		return "Delete Plan";
	}

	@Override
	protected boolean initialize(Object element) {
		boolean result = false;
		if(element instanceof IFile){
			toDeleteFile = (IFile)element;
			if(toDeleteFile.getFileExtension().equals("pml"))
				result = true;
		}
		return result;
	}

	private Collection<Setting> getUsages() {
		if(usages == null){
			// Load all plans which resist in the workspace.
			Set<IFile> planFiles = PlanEditorUtils.collectAllFilesWithExtension("pml");
			
			// Our already loaded plan is also in the set. But the resourceset will
			// detect, that the resource is already loaded
			for(IFile file : planFiles){
				editingDomain.load(file);
			}
			
			usages = EcoreUtil.UsageCrossReferencer.find(getPlanToDelete(), editingDomain.getResourceSet());
		}
		return usages;
	}

	private Plan getPlanToDelete() {
		if(planToDelete == null){
			// Load the plan we wanna delete
			Resource planToDeleteResource = editingDomain.load(toDeleteFile);
			PlanEditorUtils.getRemovedResourcesUtil().add(planToDeleteResource);
			planToDelete = (Plan)planToDeleteResource.getContents().get(0);
		}
		return planToDelete;
	}

}
