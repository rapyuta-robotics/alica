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
package de.uni_kassel.vs.cn.planDesigner.ui.policies;

import java.util.Collections;
import java.util.Set;

import org.eclipse.draw2d.geometry.Rectangle;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.util.TransactionUtil;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.EditPolicy;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.commands.UnexecutableCommand;
import org.eclipse.gef.editpolicies.NonResizableEditPolicy;
import org.eclipse.gef.requests.CreateRequest;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.IModelExclusionAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CommandWrap4EMF;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CreateUIExtensionCommmand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.SetNameAndDirectEditCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.SetUIExtensionCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.PlanEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public class PlanXYLayoutEditPolicy extends DesignerXYLayoutEditPolicy{

	@Override
	protected EditPolicy createChildEditPolicy(EditPart child)
	{
		return new NonResizableEditPolicy();
	}
	
	@Override
	protected Command getCreateCommand(CreateRequest req) {
		Command wrap = UnexecutableCommand.INSTANCE;
	
		EditPart hostEditPart = getHost();
	
		// We only know how to build children in EObject containers
		if(hostEditPart.getModel() instanceof EObject){
			if(!(req.getNewObjectType() instanceof EClass)){
				System.err.println("Creation of type \"" +req.getNewObjectType() +"\" is not supported by the " + getClass().getName() + "!");
				return wrap;
			}
			
			EObject newChild = (EObject)req.getNewObject();
		
			EObject parent = (EObject)hostEditPart.getModel();
			PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain)TransactionUtil.getEditingDomain(parent);
			
			IModelExclusionAdapter exclusionAdapter = (IModelExclusionAdapter)hostEditPart.getAdapter(IModelExclusionAdapter.class);
			Set<String> exclusionSet = null;
			if(exclusionAdapter != null)
				exclusionSet = exclusionAdapter.getExclusionClasses();
			
			// Only create the command if the child isn't in the exclusion set
			if((exclusionSet == null || !exclusionSet.contains(newChild.eClass().getName())) 
					&&  newChild.eClass() != AlicaPackage.eINSTANCE.getTask()  
					&&  newChild.eClass() != AlicaPackage.eINSTANCE.getPlan() 
					&&  newChild.eClass() != AlicaPackage.eINSTANCE.getPostCondition()){
			//if(exclusionSet == null || (exclusionSet != null && !exclusionSet.contains(newChild.eClass()))){
				CompoundCommand compound = new CompoundCommand(0);
				
				
				PlanEditor editor = PlanEditorUtils.getPlanEditor(getHost());
				compound.append(new CreateUIExtensionCommmand(editingDomain,editor.getUIExtensionMap(), newChild));
				
				// If the user wants to create an EntryPoint we also create a DefaultTask for that point
				if(newChild.eClass() == AlicaPackage.eINSTANCE.getEntryPoint()){
					// Get the default task from the repository
					CommonUtils.getTaskRepository(editingDomain, true);
					Task defaultTask = CommonUtils.getDefaultTaskFromTaskRepository(editingDomain);
					
					compound.append(SetCommand.create(
							editingDomain, 
							newChild, 
							AlicaPackage.eINSTANCE.getEntryPoint_Task(), 
							defaultTask));
				}
				
				if(newChild.eClass().equals(AlicaPackage.eINSTANCE.getState())){
					compound.append(CreateChildCommand.create(editingDomain, parent, new CommandParameter(parent, AlicaPackage.eINSTANCE.getPlan_States(), newChild), Collections.emptyList()));
				} else if(newChild.eClass().equals(AlicaPackage.eINSTANCE.getSynchronisation())){
					compound.append(CreateChildCommand.create(editingDomain, parent, new CommandParameter(parent, AlicaPackage.eINSTANCE.getPlan_Synchronisations(), newChild), Collections.emptyList()));
				} else if(newChild.eClass().equals(AlicaPackage.eINSTANCE.getTransition())){
					compound.append(CreateChildCommand.create(editingDomain, parent, new CommandParameter(parent, AlicaPackage.eINSTANCE.getPlan_Transitions(), newChild), Collections.emptyList()));
				} else if(newChild.eClass().equals(AlicaPackage.eINSTANCE.getEntryPoint())){
					compound.append(CreateChildCommand.create(editingDomain, parent, new CommandParameter(parent, AlicaPackage.eINSTANCE.getPlan_EntryPoints(), newChild), Collections.emptyList()));
				} else if(newChild.eClass().equals(AlicaPackage.eINSTANCE.getSuccessState())){
					compound.append(CreateChildCommand.create(editingDomain, parent, new CommandParameter(parent, AlicaPackage.eINSTANCE.getPlan_States(), newChild), Collections.emptyList()));
				} else if(newChild.eClass().equals(AlicaPackage.eINSTANCE.getFailureState())){
					compound.append(CreateChildCommand.create(editingDomain, parent, new CommandParameter(parent, AlicaPackage.eINSTANCE.getPlan_States(), newChild), Collections.emptyList()));
				} else if(newChild.eClass().equals(AlicaPackage.eINSTANCE.getPreCondition())){
					compound.append(CreateChildCommand.create(editingDomain, parent, new CommandParameter(parent, AlicaPackage.eINSTANCE.getAbstractPlan_Conditions(), newChild), Collections.emptyList()));
				} else if(newChild.eClass().equals(AlicaPackage.eINSTANCE.getRuntimeCondition())){
					compound.append(CreateChildCommand.create(editingDomain, parent, new CommandParameter(parent, AlicaPackage.eINSTANCE.getAbstractPlan_Conditions(), newChild), Collections.emptyList()));
				} else if(newChild.eClass().equals(AlicaPackage.eINSTANCE.getPostCondition())){
					compound.append(CreateChildCommand.create(editingDomain, parent, new CommandParameter(parent, AlicaPackage.eINSTANCE.getAbstractPlan_Conditions(), newChild), Collections.emptyList()));
				} 
				
				Rectangle constraint = (Rectangle)getConstraintFor(req);
				compound.append(new SetUIExtensionCommand(newChild,constraint,editingDomain,editor.getUIExtensionMap()));
				
				compound.append(new SetNameAndDirectEditCommand(editingDomain, newChild, "New" +newChild.eClass().getName(), getHost().getViewer()));
				
				wrap = new CommandWrap4EMF(compound);

			} 
		}
		
		return wrap;
	}
	
}
