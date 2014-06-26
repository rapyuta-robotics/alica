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

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Point;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Tree;
import org.eclipse.swt.widgets.TreeItem;

import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;

public class BehaviourDeletionDialog extends MessageDialog {
	
	class UsageTreeContentProvider implements ITreeContentProvider{
		
		Map<EObject, Collection<EStructuralFeature.Setting>> usages;
		
		Map<BehaviourConfiguration, List<Plan>> configPlanMapping; 
		
		Map<BehaviourConfiguration, List<State>> configStatesMapping;
		
		public Object[] getChildren(Object parentElement) {
			if(parentElement instanceof BehaviourConfiguration){
				return configPlanMapping.get(configPlanMapping).toArray();
			}else if(parentElement instanceof Plan){
				List<State> relevantStates = new ArrayList<State>(((Plan)parentElement).getStates());
				relevantStates.retainAll(configStatesMapping.get(parentElement));
				return relevantStates.toArray();
			}
				return null;
		}

		public Object getParent(Object element) {
//			if(element instanceof Plan){
//				
//			}else if(element instanceof State){
//				return ((State)element).eContainer();
//			}else
//				return null;
			return null;
		}

		public boolean hasChildren(Object element) {
			return element instanceof BehaviourConfiguration || element instanceof Plan;
		}

		public Object[] getElements(Object inputElement) {
			Map<EObject, Collection<EStructuralFeature.Setting>> usages = 
				(Map<EObject, Collection<EStructuralFeature.Setting>>)inputElement;
			
			return usages.keySet().toArray();
		}

		public void dispose() {
			usages = null;
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
			usages = (Map<EObject, Collection<EStructuralFeature.Setting>>)newInput;
			
			
		}
		
	}
	
	private Map<EObject, Collection<EStructuralFeature.Setting>> usages;
	
	private static final String msg = "The item you are trying to delete is used in other files. If you delete " +
			"the item it will be deleted from all files which are using this item. Do you really want to proceed?";
	
	public BehaviourDeletionDialog(Shell parent, Map<EObject, Collection<EStructuralFeature.Setting>> usages) {
		super(parent, "Confirm delete", null, // accept
                // the
                // default
                // window
                // icon
				msg, WARNING, new String[] { IDialogConstants.OK_LABEL, IDialogConstants.CANCEL_LABEL }, 0);
		this.usages = usages;
	}
	
	@Override
	protected Control createCustomArea(Composite parent) {
		Composite comp = new Composite(parent, SWT.BORDER);
		comp.setLayout(new FillLayout());
		comp.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
		
		// It's easier to use a simple tree instead of a treeViewer cause 
		// implementing the contentProviders is difficult because we 
		// don't have a simple Tree to display
		Tree tree = new Tree(comp, SWT.NONE);
		prepareTree(tree);

		return comp;
	}
	
	@Override
	protected Point getInitialSize() {
		return new Point(500,350);
	}
	
	private void prepareTree(Tree tree){
		// For each key create a TreeItem which represents the BehaviourConfiguration
		for(EObject eo : usages.keySet()){
			TreeItem behaviourConfigItem = new TreeItem(tree, SWT.NONE);
			behaviourConfigItem.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_BEHAVIOUR_CONFIGURATION_16));
			behaviourConfigItem.setText(((PlanElement)eo).getName());
			
			// Grouping step: all setting that are States and belong to the same plan will get into
			// the same TreeItem
			Map<Plan, List<State>> planStateMapping = new HashMap<Plan, List<State>>();
			
			for(EStructuralFeature.Setting setting : usages.get(eo)){
				// The EObject from the setting should be a state since configurations 
				// are contained in a state. We ignore any ui related stuff like ui extensions
				EObject obj = setting.getEObject();
				if(obj instanceof State){
					State s = (State)obj;
					Plan plan = s.getInPlan();
					
					// Check if there is already a mapping from this plan to other
					// states
					if(!planStateMapping.containsKey(plan))
						planStateMapping.put(plan, new ArrayList<State>());
					
					planStateMapping.get(plan).add(s);
				}
			}
			
			for(Plan p : planStateMapping.keySet()){
				// Create a TreeItem for each plan
				TreeItem planItem = new TreeItem(behaviourConfigItem, SWT.NONE);
				planItem.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_PLAN_16));
				planItem.setText(p.getName());
				for(State s : planStateMapping.get(p)){
					// Create a StateItem where the above configuration is contained
					TreeItem stateItem = new TreeItem(planItem, SWT.NONE);
					stateItem.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_STATE_16));
					stateItem.setText(s.getName());
				}
			}
		}
	}
}
