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
package de.uni_kassel.vs.cn.planDesigner.ui;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.application.ActionBarAdvisor;
import org.eclipse.ui.application.IActionBarConfigurer;
import org.eclipse.ui.application.IWorkbenchWindowConfigurer;
import org.eclipse.ui.application.WorkbenchWindowAdvisor;
import org.eclipse.ui.internal.dialogs.WorkbenchWizardElement;
import org.eclipse.ui.internal.wizards.AbstractExtensionWizardRegistry;
import org.eclipse.ui.wizards.IWizardCategory;
import org.eclipse.ui.wizards.IWizardDescriptor;

public class PlanDesignerWorkbenchWindowAdvisor extends WorkbenchWindowAdvisor {

	public PlanDesignerWorkbenchWindowAdvisor(IWorkbenchWindowConfigurer configurer) {
		super(configurer);
	}

	@Override
	public void preWindowOpen() {
		IWorkbenchWindowConfigurer configurer = getWindowConfigurer();

		configurer.setShowCoolBar(true);
		configurer.setShowMenuBar(true);
		configurer.setShowFastViewBars(true);
		configurer.setShowProgressIndicator(true);
		configurer.setShowStatusLine(true);
		configurer.setShowPerspectiveBar(true);

//				IWorkbenchActivitySupport workbenchActivitySupport = PlatformUI.getWorkbench().getActivitySupport();
//				IActivityManager activityManager = workbenchActivitySupport.getActivityManager();
//		
//				Set enabledActivityIds = new HashSet(activityManager.getEnabledActivityIds());
//				
//				if (enabledActivityIds.add("de.uni_kassel.vs.cn.planDesigner.ui.textEditorActivity"))
//				  workbenchActivitySupport.setEnabledActivityIds(enabledActivityIds);

	}

	@Override
	public ActionBarAdvisor createActionBarAdvisor(IActionBarConfigurer configurer) {
		return new PlanDesignerActionBarAdvisor(configurer);
	}
	@Override 
	public void postWindowOpen(){
		AbstractExtensionWizardRegistry wizardRegistry = (AbstractExtensionWizardRegistry)PlatformUI.getWorkbench().getNewWizardRegistry();
		IWizardCategory[] categories = PlatformUI.getWorkbench().getNewWizardRegistry().getRootCategory().getCategories();
		for(IWizardDescriptor wizard : getAllWizards(categories)){
			if(wizard.getCategory().getId().matches("org.eclipse.jdt.ui.java") || wizard.getCategory().getId().matches("org.eclipse.emf.mwe.ui.newWizards") || wizard.getDescription().contains("scrapbook")){
				WorkbenchWizardElement wizardElement = (WorkbenchWizardElement) wizard;
				wizardRegistry.removeExtension(wizardElement.getConfigurationElement().getDeclaringExtension(), new Object[]{wizardElement});
			}else if(wizard.getCategory().getId().matches("org.eclipse.ui.Basic")){
				if(!wizard.getDescription().contains("folder resource") && !wizard.getDescription().contains("project resource")){
					WorkbenchWizardElement wizardElement = (WorkbenchWizardElement) wizard;
					wizardRegistry.removeExtension(wizardElement.getConfigurationElement().getDeclaringExtension(), new Object[]{wizardElement});
				}
			}
			
		}
	}

	private IWizardDescriptor[] getAllWizards(IWizardCategory[] categories) {
		List<IWizardDescriptor> results = new ArrayList<IWizardDescriptor>();
		for(IWizardCategory wizardCategory : categories){
			results.addAll(Arrays.asList(wizardCategory.getWizards()));
			results.addAll(Arrays.asList(getAllWizards(wizardCategory.getCategories())));
		}
		return results.toArray(new IWizardDescriptor[0]);
	}


}
