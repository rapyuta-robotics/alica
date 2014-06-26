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

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.Assert;
import org.eclipse.core.runtime.IPath;
import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.edit.command.RemoveCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.gef.DefaultEditDomain;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.EditPartViewer;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.InternalRoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.alica.Node;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.Role;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleDefinitionSet;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleSet;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleTaskMapping;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskWrapper;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.RolesetEditor;
import de.uni_kassel.vs.cn.planDesigner.ui.exception.PlanNotFoundException;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.TaskGraphEditPart;

public class RolesetEditorUtils extends CommonUtils{
	
	/**
	 * Convenience method to get the editor from a given editPart;
	 * @return
	 */
	public static RolesetEditor getRolesetEditor(EditPart part){
		Assert.isNotNull(part);
		return ((RolesetEditor)((DefaultEditDomain)part.getViewer().getEditDomain())
				.getEditorPart());
	}

	/**
	 * Looks in the workspace for a plan with the given ID. Throws PlanNotFoundException if
	 * the plan could not be found.
	 * @param id
	 * @return
	 */
	public static Plan findPlan(PMLTransactionalEditingDomain domain, long id) throws PlanNotFoundException{
		Plan masterPlan = null;
		
		Set<IFile> planFiles = CommonUtils.collectAllFilesWithExtension("pml");
		for(IFile file : planFiles){
			Resource r = domain.load(file);
			Plan p = (Plan)r.getContents().get(0);
			if(p.getId() == id){
				masterPlan = p;
				break;
			}
		}
		
		if(masterPlan == null)
			throw new PlanNotFoundException("The plan with ID " +id +" couldnt be found in the workspace!");
		
		return masterPlan;
	}
	
	public static void refreshTaskGraphUI(TaskGraph graph, EditPartViewer viewer){
		// Get the editPart which belongs to the graph
		TaskGraphEditPart part = (TaskGraphEditPart)viewer.getEditPartRegistry().get(graph);
		part.refresh();
	}

	
	/**
	 * This transforms the InternalTaskGraphMappings back to the roleset format. 
	 * @param domain
	 * @param taskGraph
	 * @param roleSet
	 */
	public static void transformGraphToRoleset(PMLTransactionalEditingDomain domain, TaskGraph taskGraph,	RoleSet roleSet) {
		final Map<Role, RoleTaskMapping> roleToRoleTaskMapping = new HashMap<Role, RoleTaskMapping>();
		for(RoleTaskMapping mapping : roleSet.getMappings()){
			roleToRoleTaskMapping.put(mapping.getRole(), mapping);
		}
		
		CompoundCommand cmp = new CompoundCommand();
		for(Node n : taskGraph.getNodes()){
			if(n.eClass() == AlicaPackage.eINSTANCE.getTaskWrapper()){
				final TaskWrapper wrapper = (TaskWrapper)n;
				for(final InternalRoleTaskMapping mapping : wrapper.getMappings()){
					cmp.append(new RecordingCommand(domain){
						@Override
						protected void doExecute() {
							roleToRoleTaskMapping.get(
									mapping.getRole()).getTaskPriorities().put(
											new Long(wrapper.getTask().getId()),mapping.getPriority());
						}
					});
					
				}
				
			}
		}
		
		domain.getCommandStack().execute(cmp);
	}

	/**
	 * This method synchronizes the given roleset with the global defined roles.
	 * That includes removing roles from the roleset which are no longer globally 
	 * defined and adding new global defined roles to the roleset.
	 * 
	 * <p>After calling this method the roleset is in sync with the roles, globally defined.</p>
	 * @param roleSet
	 * @return
	 */
	public static Command synchronizeRoles(RolesetEditor editor) {
		CompoundCommand cmp = new CompoundCommand("Synchronized roleset");
		
		// Try to open the RoleDefinitionSetFile
		IPath path = CommonUtils.getRoleDefinitionPath();
		
		IWorkspaceRoot root = ResourcesPlugin.getWorkspace().getRoot();
		IFile roleDefFile = root.getFile(path);
		
		if(roleDefFile.exists()){
			// Ensure to load the roledefintion into the same resource
			// set to make make sure that role objects are equal
			ResourceSet resourceSet = editor.getRoleSet().eResource().getResourceSet();
			Resource globalRoleResource = resourceSet.getResource(URI.createPlatformResourceURI(path.toString(),
					true), true);
			
			// Get the global role definition set
			RoleDefinitionSet rdefset = (RoleDefinitionSet)globalRoleResource.getContents().get(0);
			
			// Get the TaskGraph
			final TaskGraph graph = editor.getTaskGraph();
			
			// Get the roleset from the editor
			final RoleSet roleSet = editor.getRoleSet();
			
			// Create a mapping from a role to the mapping for easier access
			Map<Role, RoleTaskMapping> roleToRoleTaskMapping = new HashMap<Role, RoleTaskMapping>();
			for(RoleTaskMapping map : roleSet.getMappings()){
				roleToRoleTaskMapping.put(map.getRole(), map);
			}
			
			// Check for new roles
			for(final Role r : rdefset.getRoles()){
				// Check if the mapping contains the role
				if(!roleToRoleTaskMapping.containsKey(r)){
					// We found a new role, so add a command
					// which adds the new role to the roleset
					// and generate a default taskpriority for
					// each task in the graph
					cmp.append(new RecordingCommand(editor.getEditingDomain()){
						@Override
						protected void doExecute() {
							RoleTaskMapping roleTaskMapping = AlicaFactory.eINSTANCE.createRoleTaskMapping();
							roleTaskMapping.setRole(r);
							for(Node n : graph.getNodes()){
								if(n.eClass() == AlicaPackage.eINSTANCE.getTaskWrapper()){
									roleTaskMapping.getTaskPriorities().put(((TaskWrapper)n).getTask().getId(), 0.5);
								}
							}
							roleSet.getMappings().add(roleTaskMapping);
						}
					});
				}
			}
			
			// Check for removed roles
			for(Role r : roleToRoleTaskMapping.keySet()){
				// Check if the role is still contained in
				// the global roleset
				if(!rdefset.getRoles().contains(r)){
					// We found a role which was deleted, so remove it 
					// from the roleset
					RoleTaskMapping mappingToRemove = roleToRoleTaskMapping.get(r);
					cmp.append(RemoveCommand.create(
							editor.getEditingDomain(),
							roleSet,
							AlicaPackage.eINSTANCE.getRoleSet_Mappings(),
							mappingToRemove));
				}
			}
			
			// Check for new/removed Tasks. 
			// Synchronize tasks
			// At this point we assume that every mapping contains
			// the same mappings from tasks to priorities
			RoleTaskMapping firstMapping = roleSet.getMappings().get(0);
					
			// First check for new tasks
			for(Node n : graph.getNodes()){
				if(n.eClass() == AlicaPackage.eINSTANCE.getTaskWrapper()){
					Task t = ((TaskWrapper)n).getTask();
					final Long id = t.getId();
					if(!firstMapping.getTaskPriorities().keySet().contains(id)){
						for(final RoleTaskMapping mapping : roleSet.getMappings())
							cmp.append(new RecordingCommand(editor.getEditingDomain()){
								@Override
								protected void doExecute() {
									mapping.getTaskPriorities().put(id, 0.5);
								}
							});
					}
				}
			}
			
			// Second check for removed tasks
			boolean match = false;
			for(final Long taskId : firstMapping.getTaskPriorities().keySet()){
				match = false;
				for(Node n : graph.getNodes()){
					if(n.eClass() == AlicaPackage.eINSTANCE.getTaskWrapper() && 
							((TaskWrapper)n).getTask().getId() == taskId){
						match = true;
						break;
					}
				}
				if(!match){
					for(final RoleTaskMapping mapping : roleSet.getMappings())
						cmp.append(new RecordingCommand(editor.getEditingDomain()){
							@Override
							protected void doExecute() {
								mapping.getTaskPriorities().remove(taskId);
							}
						});
						
				}
			}
						
			// Remove the loaded roledefinitionfile from the resourceset
			globalRoleResource.unload();
		}else{
			// Ok - something is wrong. The gobal roledefinition file
			// is missing, so we can't synchronize - that should not happen
		}
		
		return cmp;
	}
}
