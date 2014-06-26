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

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IResourceChangeEvent;
import org.eclipse.core.resources.IResourceChangeListener;
import org.eclipse.core.resources.IResourceDelta;
import org.eclipse.core.resources.IResourceDeltaVisitor;
import org.eclipse.core.resources.WorkspaceJob;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.MultiStatus;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.InternalEObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.ecore.util.EcoreUtil;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;

public class WorkspaceChangeListener implements IResourceChangeListener {

	public void resourceChanged(final IResourceChangeEvent event) {
		if(event.getType() != IResourceChangeEvent.POST_CHANGE)
		{
			return;
		}

		new WorkspaceJob("Searching for added plans")
		{

			@Override
			public IStatus runInWorkspace(final IProgressMonitor monitor)
					throws CoreException {

				final Collection<IFile> addedFiles = new ArrayList<IFile>();
				
				monitor.beginTask("Searching for added files", 1000);
				IResourceDelta resourceDelta = event.getDelta();
				try {
					monitor.subTask("Collecting changes");
					resourceDelta.accept(new IResourceDeltaVisitor()
					{
						
						public boolean visit(IResourceDelta delta) throws CoreException {
							monitor.worked(1);
							if(delta.getKind() != IResourceDelta.ADDED)
							{
								return true;
							}
							else
							{
								if(delta.getResource() != null)
								{
									IResource resource = delta.getResource();
									
									if(resource instanceof IFile && ((IFile)resource).getFileExtension().equals("pml"))
									{
										addedFiles.add((IFile)resource);
									}
								}
								
								return false;
							}
						}
						
					});
				} catch (CoreException e) {
					return new MultiStatus(PlanDesignerActivator.PLUGIN_ID,42,"Searching of added files failed", e);
				}
				
				if(!addedFiles.isEmpty())
				{
					ResourceSet rSet = CommonUtils.getAlicaResourceSet();
					for(IFile file : addedFiles)
					{
						// Load the copied file
						Resource loaded = rSet.getResource(URI.createPlatformResourceURI(file.getFullPath().toString(),
								true), true);
						
						monitor.subTask("Generating ID's of plan " +file.getName());
						
						try {
							generateNewIDs(loaded, file);
						} catch (IOException e) {
							return new MultiStatus(PlanDesignerActivator.PLUGIN_ID,42,"Generating of ID's failed", e);
						}
						
						// TODO: Find a way to also copy the original UI-Extension file to 
						// keep UI information. At this point we don't know the original file anymore,so
						// this operation could be difficult
						
//						// Find the UI-Extension file handle for the copied file
//						IFile uiExtensionFile = CommonUtils.findUIExtensionFile(loaded);
//						
//						if(uiExtensionFile != null && idMapping != null)
//						{
//							// Copy the UI-Extension file 
////							CopyResourcesOperation copyUIExtensionOp = new CopyResourcesOperation(uiExtensionFile, )
//						}
						
						// Save the resource
						try {
							loaded.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
						} catch (IOException e) {
							return new MultiStatus(PlanDesignerActivator.PLUGIN_ID,42,"Saving file " +file +"failed!", e);
						}
						monitor.worked(1);
					}
					for(Resource r : rSet.getResources())
					{
						r.unload();
					}
					rSet.getResources().clear();
					rSet = null;
				}
				monitor.done();
				return Status.OK_STATUS;
			}

			private Map<Long, Long> generateNewIDs(Resource resource, IFile file) throws IOException, FileNotFoundException{
				Map<Long, Long> idMapping = new HashMap<Long, Long>();
				
				// Get the plan 
				Object root = resource.getContents().get(0);
				if(root != null && root instanceof Plan)
				{
					Plan rootPlan = (Plan)root;
					
					// Change the plan name
					rootPlan.setName(file.getName().substring(0,file.getName().lastIndexOf('.')));
					rootPlan.setId(rootPlan.generateID());
					TreeIterator<Object> allContents = EcoreUtil.getAllContents(rootPlan, true);
					
					// Get a list of all plan files in the workspace 
					Set<IFile> allFiles = CommonUtils.collectAllFilesWithExtension("pml","beh","pty");
					
					while(allContents.hasNext())
					{
						Object next = allContents.next();
//						System.out.println("Processing element: <" +next +">");
						if(next instanceof PlanElement)
						{
							if(((PlanElement)next).eIsProxy()) // Something in an other file
							{
								continue;
							}
							else
							{
								PlanElement element = (PlanElement)next;
								long oldID = element.getId();
								long newID = element.generateID();
								element.setId(newID);
								
								idMapping.put(oldID, newID);
								
								if(element instanceof State)
								{
									Map<AbstractPlan, AbstractPlan> resolvedPlans = new HashMap<AbstractPlan, AbstractPlan>();
									State state = (State)element;
									for(AbstractPlan plan : state.getPlans())
									{
										// Try to resolve the plan first
										EObject resolve = null;
										
										// If we still have a proxy, search in workspace
										if(plan.eIsProxy())
										{
											resolve = EcoreUtil.resolve(plan, resource);
											
											if(resolve== null || resolve.eIsProxy())
											{
												resolve = searchAndResolvePlan(allFiles, resource.getResourceSet(), plan);
											}
										}
										else
										{
											resolve = plan;
										}
										
										if(resolve != null && !resolve.eIsProxy())	// We found the plan
										{
											System.out.println("Plan <" +resolve.eResource().getURI().lastSegment() +"> resolved!");
											resolvedPlans.put(plan, (AbstractPlan)resolve);
										}
										else
										{
											throw new FileNotFoundException("Can't find file for plan <" +((InternalEObject)plan).eProxyURI().lastSegment());
										}
									}
									
									for(AbstractPlan plan : resolvedPlans.keySet())
									{
										state.getPlans().remove(plan);
										state.getPlans().add(resolvedPlans.get(plan));
									}
//									state.getPlans().clear();
//									state.getPlans().addAll(resolvedPlans);
								}
							}
						}
					}
				}
				return idMapping;
			}

			private EObject searchAndResolvePlan(Set<IFile> allPlanFilesInWorkspace, ResourceSet rSet, AbstractPlan planToResolve) {
				String fileNameToSearch = ((InternalEObject)planToResolve).eProxyURI().lastSegment();
				long id = Long.parseLong(((InternalEObject)planToResolve).eProxyURI().fragment());
				
				AbstractPlan matchedFile = null;
				
				for(IFile f : allPlanFilesInWorkspace)
				{
					if(f.getName().equals(fileNameToSearch))
					{
						matchedFile = matchesID(rSet, f, id);
						if(matchedFile != null) // We found the right file
						{
							break;
						}
					}
				}
				
				return matchedFile != null ? matchedFile : planToResolve;
			}

			private AbstractPlan matchesID(ResourceSet rSet, IFile fileToCheck, long id) {
				AbstractPlan plan = null;
				
				Resource loaded = rSet.getResource(URI.createPlatformResourceURI(fileToCheck.getFullPath().toString(),
						true), true);
				
				if(loaded != null && !loaded.getContents().isEmpty())
				{
					// FIXME: ((PlanElement)loaded.getContents().get(0)).getId() does not work 
					// for Behaviours since we need the BehaviourConfiguration and not the Behaviour!
					long id2 = ((PlanElement)loaded.getContents().get(0)).getId();
					if(id2 == id)
					{
						plan = (AbstractPlan)loaded.getContents().get(0);
					}
				}
				
				return plan;
			}
			
		}.schedule();
	}

}
