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
package de.uni_kassel.vs.cn.planDesigner.ui.actions;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.core.commands.ExecutionException;
import org.eclipse.core.resources.IContainer;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.WorkspaceJob;
import org.eclipse.core.runtime.Assert;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IPath;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.MultiStatus;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.common.util.TreeIterator;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.ecore.util.EcoreUtil.Copier;
import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.dialogs.IInputValidator;
import org.eclipse.jface.dialogs.InputDialog;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.swt.dnd.Clipboard;
import org.eclipse.swt.dnd.FileTransfer;
import org.eclipse.swt.dnd.TransferData;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.actions.CopyFilesAndFoldersOperation;
import org.eclipse.ui.actions.CopyProjectOperation;
import org.eclipse.ui.actions.SelectionListenerAction;
import org.eclipse.ui.ide.undo.CopyResourcesOperation;
import org.eclipse.ui.ide.undo.WorkspaceUndoUtil;
import org.eclipse.ui.internal.navigator.resources.plugin.WorkbenchNavigatorMessages;
import org.eclipse.ui.part.ResourceTransfer;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.impl.PmlUIExtensionModelPackageImpl;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
//import de.uni_kassel.vs.cn.planDesigner.ui.uiextensionmodel.util.PmlUIExtensionmodelResourceFactoryImpl;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;

/**
 * Copied from {@link org.eclipse.ui.internal.navigator.resources.actions.PasteAction}
 * to do some additional things (e.g. after a plan was copied).
 * @author Zenobios
 *
 */
public class PasteAction extends SelectionListenerAction
{
	private class CopyAndChangePlanIDsJob extends WorkspaceJob
	{

		private final IResource[] resourcesToCopy;
		private final IContainer destination;

		public CopyAndChangePlanIDsJob(IResource[] resourcesToCopy, IContainer destination)
		{
			super("Copy and change plan ids");
			this.resourcesToCopy = resourcesToCopy;
			this.destination = destination;
			
			PmlUIExtensionModelPackageImpl.init();
			
			// Retrieve the extension to factory map
//			Resource.Factory.Registry reg = Resource.Factory.Registry.INSTANCE;
//			Map<String, Object> m = reg.getExtensionToFactoryMap();
//			m.put("pmlex", new PmlUIExtensionModelResourceFactoryImpl());
		}

		@Override
		public IStatus runInWorkspace(IProgressMonitor monitor)
				throws CoreException
		{
			MultiStatus status = new MultiStatus(PlanDesignerActivator.PLUGIN_ID,42,"Copy plans and change IDs",null);
			IPath[] validatedFiles = validateNoNameCollisions();
			
			if(validatedFiles == null)
			{
				return Status.CANCEL_STATUS;
			}
			
			// Split plans from other resources to copy
			Map<IResource, IPath> plansToCopy = new HashMap<IResource,IPath>();
			Map<IResource, IPath> ppToCopy = new HashMap<IResource,IPath>();
			Map<IResource, IPath> otherFilesToCopy = new HashMap<IResource,IPath>();
			
			for (int i=0; i < resourcesToCopy.length; i++)
			{
				IResource resource = resourcesToCopy[i];
				
				if(resource.getFileExtension().equals("pml"))
				{
					plansToCopy.put(resource, validatedFiles[i]);
				}
				else if(resource.getFileExtension().equals("pp"))
				{
					ppToCopy.put(resource, validatedFiles[i]);
				}
				else
				{
					otherFilesToCopy.put(resource, validatedFiles[i]);
				}
			}

			// Copy all "normal" files
			CopyResourcesOperation op = new CopyResourcesOperation(
					otherFilesToCopy.keySet().toArray(new IResource[otherFilesToCopy.size()]), 
					otherFilesToCopy.values().toArray(new IPath[otherFilesToCopy.size()]), "Copy other files");
			try
			{
				PlatformUI.getWorkbench().getOperationSupport()
						.getOperationHistory().execute(op, monitor,
									WorkspaceUndoUtil.getUIInfoAdapter(shell));
			} catch (ExecutionException e)
			{
				status.add(new Status(IStatus.ERROR, PlanDesignerActivator.PLUGIN_ID, "Error while copy resources"));
			}

			IResource[] plansToCopyArray = plansToCopy.keySet().toArray(new IResource[plansToCopy.size()]);
			
			ResourceSet originalResources = CommonUtils.getAlicaResourceSet();
			ResourceSet copiedResources = CommonUtils.getAlicaResourceSet();
			
			
			for (int i = 0; i < plansToCopyArray.length; i++)
			{
				IResource currentResource = plansToCopyArray[i];
				if (currentResource.getFileExtension().equals("pml"))
				{
					Resource originalPlanResource = originalResources.getResource(URI.createPlatformResourceURI(currentResource.getFullPath().toString(),
							true), true);
					
					EObject originalPlan = originalPlanResource.getContents().get(0);
					
					Copier planCopier = new EcoreUtil.Copier();
					Plan copiedPlan = (Plan)planCopier.copy(originalPlan);
					planCopier.copyReferences();
					
					// Generate new IDs for contained elements
					Map<Long, EObject> mappedIDs = generateNewIDs(copiedPlan);
					
					// Rename the plan
					String newFileName = validatedFiles[i].lastSegment();
					copiedPlan.setName(newFileName.substring(0,newFileName.lastIndexOf(".")));

					// Prepare a resource which will hold the copy
					Resource copiedPlanResource = copiedResources.createResource(URI.createPlatformResourceURI(validatedFiles[i].toString(),
							true));
					copiedPlanResource.getContents().add(copiedPlan);

					// Load the corresponding UIExtension file if possible
					Resource originalUIResource = originalResources.getResource(URI.createPlatformResourceURI(currentResource.getFullPath().toString().concat("ex"),
							true), true);
					
					if(originalUIResource != null)
					{
						Copier uiCopier = new EcoreUtil.Copier();
						Map<EObject, PmlUiExtension> newUIMap = new HashMap<EObject, PmlUiExtension>();
						
						PmlUiExtensionMap originalUI = (PmlUiExtensionMap)originalUIResource.getContents().get(0);
						PmlUiExtensionMap copiedUI = (PmlUiExtensionMap)uiCopier.copy(originalUI);
						uiCopier.copyReferences();
						
						for (EObject mapping : originalUI.getExtension().keySet())
						{
							if(mapping instanceof PlanElement)
							{
								EObject mappedObject = mappedIDs.get(((PlanElement)mapping).getId());
								newUIMap.put(mappedObject, originalUI.getExtension().get(mapping));
							}
						}
						
						// Remove all old UI information
						copiedUI.getExtension().clear();
						
						// And put in the new ones
						for (EObject newMapping : newUIMap.keySet())
						{
							copiedUI.getExtension().put(newMapping, newUIMap.get(newMapping));
						}
						
						// Prepare a resource which will hold the copy
						Resource copiedUIResource = copiedResources.createResource(URI.createPlatformResourceURI(validatedFiles[i].toString().concat("ex"),
								true));
						copiedUIResource.getContents().add(copiedUI);
						
					}
				}
			}
			
			for (Resource resource : copiedResources.getResources())
			{
				try
				{
					resource.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
				} catch (IOException e)
				{
					status.add(new Status(IStatus.ERROR,PlanDesignerActivator.PLUGIN_ID,"Error saving resource",e));
				}
			}

			
			//copy planningproblems
			IResource[] planningProblemsToCopyArray = ppToCopy.keySet().toArray(new IResource[ppToCopy.size()]);
			
			ResourceSet originalResourcesPP = CommonUtils.getAlicaResourceSet();
			ResourceSet copiedResourcesPP = CommonUtils.getAlicaResourceSet();
			
			
			for (int i = 0; i < planningProblemsToCopyArray.length; i++)
			{
				IResource currentResource = planningProblemsToCopyArray[i];
				if (currentResource.getFileExtension().equals("pp"))
				{
					Resource originalPPResource = originalResourcesPP.getResource(URI.createPlatformResourceURI(currentResource.getFullPath().toString(),
							true), true);
					
					EObject originalPP = originalPPResource.getContents().get(0);
					
					Copier ppCopier = new EcoreUtil.Copier();
					PlanningProblem copiedPP = (PlanningProblem)ppCopier.copy(originalPP);
					ppCopier.copyReferences();
					
					// Generate new IDs for contained elements
					generateNewIDs(copiedPP);
					
					// Rename the plan
					String newFileName = validatedFiles[i].lastSegment();
					copiedPP.setName(newFileName.substring(0,newFileName.lastIndexOf(".")));

					// Prepare a resource which will hold the copy
					Resource copiedPPResource = copiedResourcesPP.createResource(URI.createPlatformResourceURI(validatedFiles[i].toString(),
							true));
					copiedPPResource.getContents().add(copiedPP);
				}
			}
			
			for (Resource resource : copiedResourcesPP.getResources())
			{
				try
				{
					resource.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
				} catch (IOException e)
				{
					status.add(new Status(IStatus.ERROR,PlanDesignerActivator.PLUGIN_ID,"Error saving resource",e));
				}
			}
						
			return status;
		}
		
		private Map<Long,EObject> generateNewIDs(AbstractPlan copiedPlan)
		{
			Map<Long,EObject> idMapping = new HashMap<Long, EObject>();
			
			// Generate a new ID for the plan
			long newId = copiedPlan.generateID();
			idMapping.put(copiedPlan.getId(), copiedPlan);
			copiedPlan.setId(newId);
			
			TreeIterator<EObject> allContents = copiedPlan.eAllContents();
			while(allContents.hasNext())
			{
				EObject next = allContents.next();
				if(next instanceof PlanElement)
				{
					PlanElement planElement = (PlanElement)next;
					newId = planElement.generateID();
					idMapping.put(planElement.getId(), planElement);
					
					planElement.setId(newId);
				}
			}
			
			return idMapping;
		}

		private IPath[] validateNoNameCollisions()
		{
			IPath[] validatedResources = new IPath[resourcesToCopy.length];
			for(int i=0; i<resourcesToCopy.length; i++)
			{
				String finalName = resourcesToCopy[i].getName();
				if(destination.findMember(finalName) != null)
				{
					finalName = getNewNameFor(finalName);
					if(finalName == null) // User canceled dialog
					{
						return null;
					}
				}
				
				validatedResources[i] = destination.getFullPath().append(finalName);
			}
			
			return validatedResources;
		}

		private String getNewNameFor(final String finalName)
		{
			final String name = finalName.substring(0,finalName.lastIndexOf("."));
			final String originalFileExtension = CommonUtils.getFileExtension(finalName);
			
			final InputDialog dialog = new InputDialog(
					shell,
					"Name conflict", 
					"Please enter a new name for \"" +finalName +"\"",
					CommonUtils.findUniqueWorkspaceName("CopyOf" +name, "."+originalFileExtension).concat(originalFileExtension),
					new IInputValidator()
					{
						public String isValid(String newText)
						{
							
							if(CommonUtils.workspaceContainsFileName(newText))
							{
								return "File already exists";
							}
							else if(!CommonUtils.getFileExtension(newText).equals(originalFileExtension))
							{
								return "File must end with " +originalFileExtension;
							}
							else
							{
								return null;
							}
						}
					});
			
			shell.getDisplay().syncExec(new Runnable()
			{

				public void run()
				{
					dialog.setBlockOnOpen(true);
					dialog.open();
				}
				
			});

			if(dialog.getReturnCode() == Dialog.OK)
			{
				return dialog.getValue();
			}
			else
			{
				return null;
			}
		}
		
	}
	
	/**
     * The id of this action.
     */
    public static final String ID = PlatformUI.PLUGIN_ID + ".PasteAction";//$NON-NLS-1$

    /**
     * The shell in which to show any dialogs.
     */
    private Shell shell;

    /**
     * System clipboard
     */
    private Clipboard clipboard;
    
    /**
     * Creates a new action.
     *
     * @param shell the shell for any dialogs
     * @param clipboard the clipboard
     */
    public PasteAction(Shell shell, Clipboard clipboard) {
        super(WorkbenchNavigatorMessages.PasteAction_Past_); 
        Assert.isNotNull(shell);
        Assert.isNotNull(clipboard);
        this.shell = shell;
        this.clipboard = clipboard;
        setToolTipText(WorkbenchNavigatorMessages.PasteAction_Paste_selected_resource_s_); 
        setId(PasteAction.ID);
        PlatformUI.getWorkbench().getHelpSystem().setHelp(this, "HelpId"); //$NON-NLS-1$
				// TODO INavigatorHelpContextIds.PASTE_ACTION);
    }
    
	/**
     * Returns the actual target of the paste action. Returns null
     * if no valid target is selected.
     * 
     * @return the actual target of the paste action
     */
    private IResource getTarget() {
        List<IResource> selectedResources = getSelectedResources();

        for (int i = 0; i < selectedResources.size(); i++) {
            IResource resource = selectedResources.get(i);

            if (resource instanceof IProject && !((IProject) resource).isOpen()) {
				return null;
			}
            if (resource.getType() == IResource.FILE) {
				resource = resource.getParent();
			}
            if (resource != null) {
				return resource;
			}
        }
        return null;
    }

    /**
     * Returns whether any of the given resources are linked resources.
     * 
     * @param resources resource to check for linked type. may be null
     * @return true=one or more resources are linked. false=none of the 
     * 	resources are linked
     */
    private boolean isLinked(IResource[] resources) {
        for (int i = 0; i < resources.length; i++) {
            if (resources[i].isLinked()) {
				return true;
			}
        }
        return false;
    }

    /**
     * Implementation of method defined on <code>IAction</code>.
     */
    public void run() {
        // try a resource transfer
        ResourceTransfer resTransfer = ResourceTransfer.getInstance();
        IResource[] resourceData = (IResource[]) clipboard
                .getContents(resTransfer);

        if (resourceData != null && resourceData.length > 0) {
            if (resourceData[0].getType() == IResource.PROJECT) {
                // enablement checks for all projects
                for (int i = 0; i < resourceData.length; i++) {
                    CopyProjectOperation operation = new CopyProjectOperation(
                            this.shell);
                    operation.copyProject((IProject) resourceData[i]);
                }
            } else {
                // enablement should ensure that we always have access to a container
                IContainer container = getContainer();

                new CopyAndChangePlanIDsJob(resourceData, container).schedule();
                
            }
            return;
        }

        // try a file transfer
        FileTransfer fileTransfer = FileTransfer.getInstance();
        String[] fileData = (String[]) clipboard.getContents(fileTransfer);

        if (fileData != null) {
            // enablement should ensure that we always have access to a container
            IContainer container = getContainer();

            CopyFilesAndFoldersOperation operation = new CopyFilesAndFoldersOperation(
                    this.shell);
            operation.copyFiles(fileData, container);
            
            
        }
    }
    
    /**
     * Returns the container to hold the pasted resources.
     */
    private IContainer getContainer() {
        List selection = getSelectedResources();
        if (selection.get(0) instanceof IFile) {
			return ((IFile) selection.get(0)).getParent();
		}
        return (IContainer) selection.get(0);
    }

    /**
     * The <code>PasteAction</code> implementation of this
     * <code>SelectionListenerAction</code> method enables this action if 
     * a resource compatible with what is on the clipboard is selected.
     * 
     * -Clipboard must have IResource or java.io.File
     * -Projects can always be pasted if they are open
     * -Workspace folder may not be copied into itself
     * -Files and folders may be pasted to a single selected folder in open 
     * 	project or multiple selected files in the same folder 
     */
    protected boolean updateSelection(IStructuredSelection selection) {
        if (!super.updateSelection(selection)) {
			return false;
		}

        final IResource[][] clipboardData = new IResource[1][];
        shell.getDisplay().syncExec(new Runnable() {
            public void run() {
                // clipboard must have resources or files
                ResourceTransfer resTransfer = ResourceTransfer.getInstance();
                clipboardData[0] = (IResource[]) clipboard
                        .getContents(resTransfer);
            }
        });
        IResource[] resourceData = clipboardData[0];
        boolean isProjectRes = resourceData != null && resourceData.length > 0
                && resourceData[0].getType() == IResource.PROJECT;

        if (isProjectRes) {
            for (int i = 0; i < resourceData.length; i++) {
                // make sure all resource data are open projects
                // can paste open projects regardless of selection
                if (resourceData[i].getType() != IResource.PROJECT
                        || ((IProject) resourceData[i]).isOpen() == false) {
					return false;
				}
            }
            return true;
        }

        if (getSelectedNonResources().size() > 0) {
			return false;
		}

        IResource targetResource = getTarget();
        // targetResource is null if no valid target is selected (e.g., open project) 
        // or selection is empty	
        if (targetResource == null) {
			return false;
		}

        // can paste files and folders to a single selection (file, folder, 
        // open project) or multiple file selection with the same parent
        List<IResource> selectedResources = getSelectedResources();
        if (selectedResources.size() > 1) {
            for (int i = 0; i < selectedResources.size(); i++) {
                IResource resource =  selectedResources.get(i);
                if (resource.getType() != IResource.FILE) {
					return false;
				}
                if (!targetResource.equals(resource.getParent())) {
					return false;
				}
            }
        }
        if (resourceData != null) {
            // linked resources can only be pasted into projects
            if (isLinked(resourceData)
                && targetResource.getType() != IResource.PROJECT
                && targetResource.getType() != IResource.FOLDER) {
				return false;
			}

            if (targetResource.getType() == IResource.FOLDER) {
                // don't try to copy folder to self
                for (int i = 0; i < resourceData.length; i++) {
                    if (targetResource.equals(resourceData[i])) {
						return false;
					}
                }
            }
            return true;
        }
        TransferData[] transfers = clipboard.getAvailableTypes();
        FileTransfer fileTransfer = FileTransfer.getInstance();
        for (int i = 0; i < transfers.length; i++) {
            if (fileTransfer.isSupportedType(transfers[i])) {
				return true;
			}
        }
        return false;
    }
}
