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

import java.io.File;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.mapping.IResourceChangeDescriptionFactory;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.OperationCanceledException;
import org.eclipse.emf.common.util.URI;
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
import org.eclipse.ltk.core.refactoring.participants.MoveParticipant;
import org.eclipse.ltk.core.refactoring.participants.ResourceChangeChecker;

import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public class MoveFileParticipant extends MoveParticipant {

	private String ext;
//	private boolean movePmlexToo;
	private IFile toMoveFile;
	private Collection<Setting> usages;
	private PlanElement toMovePlanElement;
	private Resource toMovePlanElementResource;
	private PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain) TransactionalEditingDomain.Registry.INSTANCE
	.getEditingDomain(PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);

	public MoveFileParticipant() {	}

	@Override
	protected boolean initialize(Object element) {
		boolean result = false;
		if (element instanceof IFile) {
			this.toMoveFile = (IFile) element;
			this.ext = this.toMoveFile.getFileExtension();
			result = (ext.equals("pml") || ext.equals("pty") || ext.equals("beh") || ext.equals("rset")
					|| ext.equals("rset") || ext.equals("graph") || ext.equals("pmlex"));
		}
		return result;
	}

	@Override
	public RefactoringStatus checkConditions(IProgressMonitor pm, CheckConditionsContext context)
			throws OperationCanceledException {

		RefactoringStatus result = new RefactoringStatus();
//		if (this.movePmlexToo)
//			result.addWarning("The corresponding .pmlex-File will be moved too, if you continue.");

		// We will add each resource which we modify to the deltaFactory.
		// We add those resources in which we will change references to toMovePlanElement.
		Collection<Setting> usages = getUsages();
		if (!usages.isEmpty()) {
			ResourceChangeChecker checker = (ResourceChangeChecker) context.getChecker(ResourceChangeChecker.class);
			IResourceChangeDescriptionFactory deltaFactory = checker.getDeltaFactory();
			for (Setting setting : usages) {
				EObject owner = setting.getEObject();
				Resource ownerResource = owner.eResource();
				IFile fileToChange = WorkspaceSynchronizer.getUnderlyingFile(ownerResource);
				deltaFactory.change(fileToChange);
			}
		}

		return result;
	}

	/**
	 * Determines the Collection<Setting> of all resources, which refer to the
	 * plan element, whose file are moved.
	 * 
	 * @return usages
	 */
	private Collection<Setting> getUsages() {
		if (usages == null) {
			Set<IFile> files = new HashSet<IFile>();

			/*
			 * Load everything which could reference to the file, you want to
			 * move: Move -> Load
			 * Plans (.pml) -> [.pml, .pty, .pmlex]
			 * Behaviours (.beh) -> [.pml, .pmlex]
			 * PlanTypes (.pty) -> [.pml, .pmlex]
			 * RoleDefSet (.rdefset) -> [.rset, .graph, pmlex]
			 * RolesetGraph (.graph) -> [.pmlex]
			 * UI-Arrangements (.pmlex) -> [nothing]
			 * Roleset (.rset) -> [nothing]
			 */
			if (this.ext.equals("pml")) {
				files.addAll(PlanEditorUtils.collectAllFilesWithExtension("pml", "pty", "pmlex"));
			} else if (this.ext.equals("beh") || this.ext.equals("pty")) {
				files.addAll(PlanEditorUtils.collectAllFilesWithExtension("pml", "pmlex"));
			} else if (this.ext.equals("rdefset")) {
				files.addAll(PlanEditorUtils.collectAllFilesWithExtension("rset", "graph", "pmlex"));
			} else if (this.ext.equals("graph")) {
				files.addAll(PlanEditorUtils.collectAllFilesWithExtension("pmlex"));
			}

			// The load operation is idempotent.
			for (IFile file : files) {
				editingDomain.load(file);
			}
			
			if (ext.equals("pmlex"))
				System.out.println(toMoveFile.getFullPath().toString());
			
			if (ext.equals("beh"))
			{
				// we need to search for behaviour configuration references, not for the behaviour itself
				Behaviour behaviour = (Behaviour) getPlanElementToMove();
				usages = new ArrayList<EStructuralFeature.Setting> ();	
				for (BehaviourConfiguration bc : behaviour.getConfigurations())
				{
					usages.addAll(EcoreUtil.UsageCrossReferencer.find(bc, editingDomain.getResourceSet()));					
				}
			} else {
				usages = EcoreUtil.UsageCrossReferencer.find(getPlanElementToMove(), editingDomain.getResourceSet());				
			}
		}
		return usages;
	}

	/**
	 * Determines the Plan Element, whose file should be moved.
	 * 
	 * @return toMovePlanElement
	 */
	private PlanElement getPlanElementToMove() {
		if (toMovePlanElement == null) {
			toMovePlanElement = (PlanElement) getPlanElementToMoveResource().getContents().get(0);
		}
		return toMovePlanElement;
	}

	/**
	 * Determines the Plan Element Resource, whose file should be moved.
	 * 
	 * @return planElementToMoveResource
	 */
	private Resource getPlanElementToMoveResource() {
		if (toMovePlanElementResource == null) {
			toMovePlanElementResource = editingDomain.load(toMoveFile);
		}
		return toMovePlanElementResource;
	}

	@Override
	public Change createChange(IProgressMonitor pm) throws CoreException, OperationCanceledException {
		CompositeChange cmpChange = new CompositeChange("Move File Composite Change");

		// create change for the file, which should be moved
		String newPath = ((IResource) this.getArguments().getDestination()).getFullPath().toString() + File.separator + toMoveFile.getName();
		cmpChange.add(new MoveFileChange(this.getPlanElementToMoveResource(), URI.createPlatformResourceURI(newPath, true)));
		
		// create changes for each file, which is referencing to the file which should be moved
		Collection<Setting> usages = getUsages();
		if (!usages.isEmpty()) {
			for (Setting setting : usages) {
				cmpChange.add(new SaveFileChange(setting.getEObject(), getPlanElementToMove()));
			}
		}

		return cmpChange;
	}

	@Override
	public String getName() {
		return "Move File Participant";
	}

}
