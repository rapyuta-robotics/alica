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
package de.uni_kassel.vs.cn.planDesigner.ui.wizards;

import java.io.IOException;
import java.util.Set;

import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.wizard.Wizard;

import de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLComposePlanTypeWizardPage;

public class PMLPlanTypeConfigurationWizard extends Wizard {
	
	private PMLComposePlanTypeWizardPage page;
	
	private PlanType pType;
	
	private ResourceSet rSet;
	
	private PMLTransactionalEditingDomain domain;
	
	public PMLPlanTypeConfigurationWizard(PlanType pType) {
		super();
		setWindowTitle("Configure plantype " +pType.getName());
		
		this.pType = pType;
	}
	
	@Override
	public void addPages() {
		page = new PMLComposePlanTypeWizardPage(getEditingDomain(), getResourceSet(), pType);
		addPage(page);
	}
	
	@Override
	public boolean performFinish() {
		// Save the resource where the plantype we are configuring is in
		getEditingDomain().getCommandStack().execute(new AbstractCommand(){

			@Override
			public boolean canUndo() {
				return false;
			}

			@Override
			protected boolean prepare() {
				return true;
			}

			public void execute() {
				// Get the set of plans which should now be in the planType
				Set<AnnotatedPlan> plansToAdd = page.getPlantypeViewerList();
				
				// Remove all plans from he plantype 
				pType.getPlans().clear();
				
				// Add all plans to the planType
				pType.getPlans().addAll(plansToAdd);
				
				pType.ensureParametrisationConsistency();
				
				// Save the planType
				Resource ptypeResource = pType.eResource();
				try {
					ptypeResource.save(ptypeResource.getResourceSet().getLoadOptions());
				} catch (IOException e) {
					e.printStackTrace();
				}
			}

			public void redo() {
				// Nothing to redo since we cannot undo
			}
			
		});
		
		return true;
	}
	
	private ResourceSet getResourceSet() {
		if(rSet == null)
			rSet = getEditingDomain().getResourceSet();
		
		return rSet;
	}

	private PMLTransactionalEditingDomain getEditingDomain() {
		if(domain == null)
			domain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		
		return domain;
	}

}
