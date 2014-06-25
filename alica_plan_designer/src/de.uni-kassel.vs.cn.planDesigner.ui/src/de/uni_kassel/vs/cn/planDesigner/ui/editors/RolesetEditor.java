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
package de.uni_kassel.vs.cn.planDesigner.ui.editors;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import org.eclipse.core.commands.operations.IOperationHistory;
import org.eclipse.core.commands.operations.IOperationHistoryListener;
import org.eclipse.core.commands.operations.IUndoContext;
import org.eclipse.core.commands.operations.IUndoableOperation;
import org.eclipse.core.commands.operations.ObjectUndoContext;
import org.eclipse.core.commands.operations.OperationHistoryEvent;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.runtime.IAdaptable;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.draw2d.ColorConstants;
import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.common.notify.AdapterFactory;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.edit.domain.IEditingDomainProvider;
import org.eclipse.emf.edit.provider.ComposedAdapterFactory;
import org.eclipse.emf.edit.provider.ReflectiveItemProviderAdapterFactory;
import org.eclipse.emf.edit.provider.resource.ResourceItemProviderAdapterFactory;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.RollbackException;
import org.eclipse.emf.transaction.Transaction;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.emf.workspace.IWorkspaceCommandStack;
import org.eclipse.emf.workspace.ResourceUndoContext;
import org.eclipse.gef.ContextMenuProvider;
import org.eclipse.gef.DefaultEditDomain;
import org.eclipse.gef.EditDomain;
import org.eclipse.gef.EditPartFactory;
import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.GraphicalEditPart;
import org.eclipse.gef.GraphicalViewer;
import org.eclipse.gef.commands.CommandStack;
import org.eclipse.gef.editparts.ScalableFreeformRootEditPart;
import org.eclipse.gef.editparts.ZoomManager;
import org.eclipse.gef.ui.actions.ActionRegistry;
import org.eclipse.gef.ui.actions.UpdateAction;
import org.eclipse.gef.ui.actions.ZoomInAction;
import org.eclipse.gef.ui.actions.ZoomOutAction;
import org.eclipse.gef.ui.parts.ScrollingGraphicalViewer;
import org.eclipse.jface.action.IAction;
import org.eclipse.jface.dialogs.ProgressMonitorDialog;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.ISelectionProvider;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.ui.IEditorInput;
import org.eclipse.ui.IEditorPart;
import org.eclipse.ui.IEditorSite;
import org.eclipse.ui.IFileEditorInput;
import org.eclipse.ui.ISelectionListener;
import org.eclipse.ui.IWorkbenchPart;
import org.eclipse.ui.IWorkbenchWindow;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.actions.WorkspaceModifyOperation;
import org.eclipse.ui.commands.ActionHandler;
import org.eclipse.ui.handlers.IHandlerService;
import org.eclipse.ui.part.EditorPart;
import org.eclipse.ui.views.contentoutline.IContentOutlinePage;
import org.eclipse.ui.views.properties.IPropertySheetPage;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.RoleSet;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskGraph;
import de.uni_kassel.vs.cn.planDesigner.alica.provider.AlicaItemProviderAdapterFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap;
import de.uni_kassel.vs.cn.planDesigner.ui.RolesetEditPartFactory;
import de.uni_kassel.vs.cn.planDesigner.ui.actions.AutoLayoutAction;
import de.uni_kassel.vs.cn.planDesigner.ui.actions.ExpandCollapseAllAction;
import de.uni_kassel.vs.cn.planDesigner.ui.actions.RolesetDiagramContextMenuProvider;
import de.uni_kassel.vs.cn.planDesigner.ui.actions.UpdateRoleSetAction;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CreateUIExtensionCommmand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.EMF2GEFCommandStack;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.NonUndoableCommandWrap;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.SwitchResourceContentsCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.exception.PlanNotFoundException;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.ICommandStackTabbedPropertySheetPageContributor;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.PMLTabbedPropertySheetPage;
import de.uni_kassel.vs.cn.planDesigner.ui.util.DiagramDropTargetListener;
import de.uni_kassel.vs.cn.planDesigner.ui.util.GraphDelta;
import de.uni_kassel.vs.cn.planDesigner.ui.util.OverviewOutlinePage;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.RolesetEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.TaskGraphBuilder;
import de.uni_kassel.vs.cn.planDesigner.ui.util.ToolbarComposite;
import de.uni_kassel.vs.cn.planDesigner.ui.util.UIAwareEditor;


public class RolesetEditor extends EditorPart implements ICommandStackTabbedPropertySheetPageContributor,IEditingDomainProvider, ISelectionProvider, IAdaptable, UIAwareEditor{
	

	/**
	 * The EditDomain which bundles the GraphicalViewer, the PaletteViewer
	 * the commandStack. The commandStack will be a EMF2GEFCommandStack.GEFCommandStack
	 * to work with the EMF Editing domain.
	 * 
	 */
	private EditDomain editDomain;
	
	/**
	 * The editingDomain is the EMF side. It has the connection to the model and also
	 * a commandStack which will be a EMF2GEFCommandStack.
	 */
	private PMLTransactionalEditingDomain editingDomain;
	
	/**
	 * This is the one adapter factory used for providing views of the model.
	 */
	protected ComposedAdapterFactory adapterFactory;
	
	private ActionRegistry actionRegistry;
	
	/**
	 * The GraphicalViewer
	 */
	private GraphicalViewer graphicalViewer;
	
	private Resource rolesetResource;
		
	private TaskGraph graph;
	
	private Resource uiExtensionResource;
	
	// We track dirty state by the last operation executed when saved
	private IUndoableOperation savedOperation;
	private IEditorSite siteTest;
	private IEditorInput inputTest;
	public IEditorInput  getInputTest() {
		return inputTest;
	}

	public void setInputTest(IEditorInput  file) {
		this.inputTest = file;
	}

	public IEditorSite getSiteTest() {
		return siteTest;
	}

	public void setSiteTest(IEditorSite site) {
		this.siteTest = site;
	}

	/**
	 * Resources that have been saved.
	 */
	protected Collection<Resource> savedResources = new ArrayList<Resource>();
	
	/**
	 * This keeps track of the selection of the editor as a whole.
	 */
	protected ISelection editorSelection = StructuredSelection.EMPTY;
	
	/**
	 * This keeps track of all the {@link org.eclipse.jface.viewers.ISelectionChangedListener}s that are listening to this editor.
	 */
	protected Collection<ISelectionChangedListener> selectionChangedListeners = new ArrayList<ISelectionChangedListener>();
	
	/**
	 * The selectionListener.
	 */
	private ISelectionListener selectionListener = new ISelectionListener() {
		public void selectionChanged(IWorkbenchPart part, ISelection selection) {
			setSelection(selection);
		}
	};
	
	private ObjectUndoContext undoContext;
	
	private PMLTabbedPropertySheetPage propertyPage;
	
	private TaskGraphBuilder builder;
	
	/**
	 * The history listeners listenes to the IOperationHistory and currently determines 
	 * if this editor is affected by change of the history. The editor is affected if 
	 * the operations modified resources belonging to this editor instance.
	 */
	private final IOperationHistoryListener historyListener = new IOperationHistoryListener() {
		public void historyNotification(final OperationHistoryEvent event) {
			Set<Resource> affectedResources = ResourceUndoContext.getAffectedResources(
					event.getOperation());
			
			switch(event.getEventType()) {
			case OperationHistoryEvent.DONE:
				if (affectedResources.contains(getRolesetResource()) || affectedResources.contains(getTaskGraphResource()) ||
						affectedResources.contains(getUIExtensionResource())) {
					final IUndoableOperation operation = event.getOperation();
					
					// remove the default undo context so that we can have
					//     independent undo/redo of independent resource changes
					operation.removeContext(((IWorkspaceCommandStack)
							getEditingDomain().getCommandStack()).getDefaultUndoContext());
					
					// add our undo context to populate our undo menu
					operation.addContext(getUndoContext());
					
					getSite().getShell().getDisplay().asyncExec(new Runnable() {
						public void run() {
							firePropertyChange(IEditorPart.PROP_DIRTY);
							
							if (propertyPage != null && propertyPage.getControl().isVisible() && propertyPage.getCurrentTab() != null) {
								propertyPage.refresh();
							}
						}
					});
				}
				break;
			case OperationHistoryEvent.UNDONE:
			case OperationHistoryEvent.REDONE:
				if (affectedResources.contains(getRolesetResource()) || affectedResources.contains(getTaskGraphResource()) ||
						affectedResources.contains(getUIExtensionResource())) {
					getSite().getShell().getDisplay().asyncExec(new Runnable() {
						public void run() {
							firePropertyChange(IEditorPart.PROP_DIRTY);
							
							if (propertyPage != null && propertyPage.getControl().isVisible() && propertyPage.getCurrentTab() != null) {
								propertyPage.refresh();
							}
						}
					});
				}
				break;
			}
		}
	};

	public RolesetEditor() {
		super();
		createActions();
		undoContext = new ObjectUndoContext(this, PlanDesignerConstants.ROLESET_EDITOR_ID);
	}
	
	/**
	 * Creates actions and adds them to the actionRegistry
	 */
	private void createActions(){
		ActionRegistry reg = getActionRegistry();
		reg.registerAction(new AutoLayoutAction(this));
		reg.registerAction(new UpdateRoleSetAction(this, rolesetResource));
		reg.registerAction(new ExpandCollapseAllAction(this,false));
		reg.registerAction(new ExpandCollapseAllAction(this,true));
	}
	
	/**
	 * Updates all UpdateAction in the actionRegistry
	 * @param actionIds the list of IDs to update
	 */
	protected void updateActions() {
		ActionRegistry registry = getActionRegistry();
		Iterator<IAction> iter = registry.getActions();
		while (iter.hasNext()) {
			IAction action = iter.next();
			if (action instanceof UpdateAction)
				((UpdateAction)action).update();
		}
	}

	@Override
	public void doSave(IProgressMonitor monitor) {
		WorkspaceModifyOperation operation =
			new WorkspaceModifyOperation() {
			// This is the method that gets invoked when the operation runs.
			public void execute(IProgressMonitor monitor) {
				try {
					// Do the back transformation
					RolesetEditorUtils.transformGraphToRoleset(getEditingDomain(),getTaskGraph(),getRoleSet());
					// Save the roleset
					getRolesetResource().save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
					savedResources.add(getRolesetResource());
					
					// Save the .graph 
					getTaskGraphResource().save(null);
					savedResources.add(getTaskGraphResource());
					
					// Save the UI Extension
					// Remove any corrupt extensions
					removeCorruptExtensions();
					getUIExtensionResource().save(null);
					savedResources.add(getUIExtensionResource());
				}
				catch (Exception exception) {
					exception.printStackTrace();
				}
			}
		};

		try {
			new ProgressMonitorDialog(getSite().getShell()).run(true, false, operation);
			
			// Refresh the necessary state.
			//
			//We record the last operation executed when saved.
			savedOperation = getOperationHistory().getUndoOperation(getUndoContext());
//			getEMFCommandStack().saveIsDone();
			firePropertyChange(IEditorPart.PROP_DIRTY);
		} catch (InvocationTargetException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
	
	/**
	 * Removes corrupt extensions which would be saved within the extension file.
	 * An extension is corrupt if the corresponding EObject is not contained in 
	 * a resource and due to that could not be loaded after opening the extension
	 * file again.
	 */
	private void removeCorruptExtensions() throws RollbackException, InterruptedException{
		getEMFCommandStack().execute(new RecordingCommand(getEditingDomain()){
			@Override
			protected void doExecute() {
				PmlUiExtensionMap myUIMap = getUIExtensionMap();
				// Check if extension has a corresponding EObject
				Set<EObject> keys = myUIMap.getExtension().keySet();
				List<EObject> toRemove = new ArrayList<EObject>();
				for(EObject e : keys){
					if(e == null || e.eResource() == null)
						toRemove.add(e);
				}
				for(EObject e : toRemove)
					myUIMap.getExtension().remove(e);
			}
		},Collections.singletonMap(
	            Transaction.OPTION_UNPROTECTED, Boolean.TRUE));
		
	}

	@Override
	public void doSaveAs() {
		// We do not support save as for now
	}

	@Override
	public void init(IEditorSite site, IEditorInput input)
			throws PartInitException {
		IFile file = ((IFileEditorInput)input).getFile();
		this.setInputTest(input);
		this.setSiteTest(site);
		
		if(file == null)
			throw new PartInitException("File for " +input +" not found");
		
		// Load the file with the editingDomain
		Resource resource = getEditingDomain().load(file);
		
		if(resource != null){
			setRolesetResource(resource);
//			checkForUnresolvableReferences(resource, true);
		} else
			throw new PartInitException("Could not load the plan");
		
		// load the ui extension resource
		Resource uiExt = getEditingDomain().loadExtensionResource(resource);
		setUIExtensionResource(uiExt);
				
//		TaskGraphBuilder builder = new TaskGraphBuilder(getEditingDomain(), resource);
		builder = new TaskGraphBuilder(getEditingDomain(), resource);
		setBuilder(builder);
		
		// First, build a fresh graph, based on the rolesets usableWithPlanID attribute
		TaskGraph freshGraph;
		try {
			freshGraph = builder.createFreshTaskGraph();
		} catch (PlanNotFoundException e) {
			throw new PartInitException(e.getMessage());
		}
		
		// Tell the build to try loading a saved graph
		TaskGraph savedGraph = builder.getSavedGraph();
		
		TaskGraph graphToUse = null;
		
		CompoundCommand cmp = new CompoundCommand("Adjusting roleset");
		
		// Get the resource where to save the graph. At this point the contents
		// of the resource can be the saved graph OR the content can be empty
		// (since there is saved graph)
		Resource graphResource = builder.getGraphResource();
		if(savedGraph != null){
			// There is an old graph so check if there is a delta
			Set<GraphDelta> delta = TaskGraphBuilder.getGraphDelta(savedGraph, freshGraph);
			
			if(!delta.isEmpty()){
				// There is a difference between two graphs...
				cmp.appendIfCanExecute(TaskGraphBuilder.handleDeltaForRoleset(getEditingDomain(),resource, delta));
				
				// Switch the graphs to use the new one
				graphToUse = freshGraph;
				cmp.append(new SwitchResourceContentsCommand(
						getEditingDomain(),
						graphResource,
						savedGraph,
						graphToUse));
				
				savedGraph = null;
			}else{
				// There is no difference, use the saved graph
				graphToUse = savedGraph;
				freshGraph = null;
			}
		}else{
			graphToUse = freshGraph;
			// At this point we have to ensure that the freshGraph is contained
			// in a resource. So we set the freshGraph as contents for the graphresource
			cmp.append(new SwitchResourceContentsCommand(
					getEditingDomain(),
					graphResource,
					null,
					graphToUse));
		
		}
		
		// Set the graph to use
		setGraph(graphToUse);
		// Synchronize the roleset. 
		Command syncRoleSetCommand = RolesetEditorUtils.synchronizeRoles(this); 
		cmp.appendIfCanExecute(syncRoleSetCommand);
		
		// Store site and Input
		setSite(site);
		setInput(input);
//		site.getPage().addPartListener(partListener);
		
		setPartName(file.getName());

		// Add selection change listener
		getSite().getWorkbenchWindow().getSelectionService()
				.addSelectionListener(getSelectionListener());

		// Start listen to the operation history
		getOperationHistory().addOperationHistoryListener(historyListener);
		
		// Execute all added commands
		getEMFCommandStack().execute(new NonUndoableCommandWrap(cmp.unwrap()));
		// This is a hack: Refreshing the taskwrappers occurs every time, 
		// a roleset is opened but we don't want the editor to get dirty
		// So stop listening...
		getOperationHistory().removeOperationHistoryListener(historyListener);
		// ... fill the TaskWrappers with roleTaskMappings from the rolesetresource ...
		Command refreshTaskWrapperCommand = TaskGraphBuilder.refreshTaskWrappers(getEditingDomain(),graphToUse, getRoleSet());
		// .. execute the command ...
		getEMFCommandStack().execute(refreshTaskWrapperCommand);
		// ... and resume listening
		getOperationHistory().addOperationHistoryListener(historyListener);
		
		
		if(getGraphicalViewer() != null){
			getGraphicalViewer().setContents(getTaskGraph());
		}
		
//		workspaceSynchronizer = new WorkspaceSynchronizer( getEditingDomain(), createSynchronizationDelegate());
		
	}
	
	
	public void updatePlan()
			throws PartInitException {
		IFile file = ((IFileEditorInput)this.getInputTest()).getFile();
	
		if(file == null)
			throw new PartInitException("File for " + this.getInputTest() +" not found");
		
		// Load the file with the editingDomain
		Resource resource = getEditingDomain().load(file);
		
		if(resource != null){
			setRolesetResource(resource);
//			checkForUnresolvableReferences(resource, true);
		} else
			throw new PartInitException("Could not load the plan");
		
		// load the ui extension resource
		Resource uiExt = getEditingDomain().loadExtensionResource(resource);
		setUIExtensionResource(uiExt);
				
//		TaskGraphBuilder builder = new TaskGraphBuilder(getEditingDomain(), resource);
		builder = new TaskGraphBuilder(getEditingDomain(), resource);
		setBuilder(builder);
		
		// First, build a fresh graph, based on the rolesets usableWithPlanID attribute
		TaskGraph freshGraph;
		try {
			freshGraph = builder.createFreshTaskGraph();
		} catch (PlanNotFoundException e) {
			throw new PartInitException(e.getMessage());
		}
		
		// Tell the build to try loading a saved graph
		TaskGraph savedGraph = builder.getSavedGraph();
		
		TaskGraph graphToUse = null;
		
		CompoundCommand cmp = new CompoundCommand("Adjusting roleset");
		
		// Get the resource where to save the graph. At this point the contents
		// of the resource can be the saved graph OR the content can be empty
		// (since there is saved graph)
		Resource graphResource = builder.getGraphResource();
		if(savedGraph != null){
			// There is an old graph so check if there is a delta
			Set<GraphDelta> delta = TaskGraphBuilder.getGraphDelta(savedGraph, freshGraph);
			
			if(!delta.isEmpty()){
				// There is a difference between two graphs...
				cmp.appendIfCanExecute(TaskGraphBuilder.handleDeltaForRoleset(getEditingDomain(),resource, delta));
				
				// Switch the graphs to use the new one
				graphToUse = freshGraph;
				cmp.append(new SwitchResourceContentsCommand(
						getEditingDomain(),
						graphResource,
						savedGraph,
						graphToUse));
				
				savedGraph = null;
			}else{
				// There is no difference, use the saved graph
				graphToUse = savedGraph;
				freshGraph = null;
			}
		}else{
			graphToUse = freshGraph;
			// At this point we have to ensure that the freshGraph is contained
			// in a resource. So we set the freshGraph as contents for the graphresource
			cmp.append(new SwitchResourceContentsCommand(
					getEditingDomain(),
					graphResource,
					null,
					graphToUse));
		
		}
		
		// Set the graph to use
		setGraph(graphToUse);

		// Store site and Input
		setSite(this.getSiteTest());
		setInput(this.getInputTest());
		
		setPartName(file.getName());

		// Add selection change listener
		getSite().getWorkbenchWindow().getSelectionService()
				.addSelectionListener(getSelectionListener());

		// Start listen to the operation history
		getOperationHistory().addOperationHistoryListener(historyListener);
		
		// Execute all added commands
		getEMFCommandStack().execute(new NonUndoableCommandWrap(cmp.unwrap()));
		// This is a hack: Refreshing the taskwrappers occurs every time, 
		// a roleset is opened but we don't want the editor to get dirty
		// So stop listening...
		getOperationHistory().removeOperationHistoryListener(historyListener);
		// ... fill the TaskWrappers with roleTaskMappings from the rolesetresource ...
		Command refreshTaskWrapperCommand = TaskGraphBuilder.refreshTaskWrappers(getEditingDomain(),graphToUse, getRoleSet());
		// .. execute the command ...
		getEMFCommandStack().execute(refreshTaskWrapperCommand);
		// ... and resume listening
		getOperationHistory().addOperationHistoryListener(historyListener);
		
		
		if(getGraphicalViewer() != null){
			getGraphicalViewer().setContents(getTaskGraph());
		}
	}
	
	public TaskGraphBuilder getBuilder() {
		return builder;
	}

	public void setBuilder(TaskGraphBuilder builder) {
		this.builder = builder;
	}

	public IOperationHistory getOperationHistory() {
		return ((IWorkspaceCommandStack) getEditingDomain().getCommandStack()).getOperationHistory();
	}
	
	public RoleSet getRoleSet(){
		return (RoleSet)getRolesetResource().getContents().get(0);
	}
	
	@Override
	public void dispose() {
		// Remove selection listener
		getSite().getWorkbenchWindow().getSelectionService()
				.removeSelectionListener(getSelectionListener());

		// Remove the operation history stuff
		getOperationHistory().removeOperationHistoryListener(historyListener);
		getOperationHistory().dispose(getUndoContext(), true, true, true);
		
		// Clean up by removing the loaded resources from the shared resourceSet
		getUIExtensionResource().eAdapters().clear();
		getTaskGraphResource().eAdapters().clear();
		getRolesetResource().eAdapters().clear();
		getEditingDomain().getResourceSet().getResources().remove(getUIExtensionResource());
		getEditingDomain().getResourceSet().getResources().remove(getTaskGraphResource());
		getEditingDomain().getResourceSet().getResources().remove(getRolesetResource());
		
		// TODO: Remove the partListener if we do support saveAs, cause otherwise there will be
		// no registered partListener @see setSite()!
//		getSite().getPage().removePartListener(partListener);
		
		adapterFactory.dispose();
		
		super.dispose();
	}
	
	
	
	@Override
	public Object getAdapter(Class adapter) {
		// Handle common GEF elements
		if (adapter == GraphicalViewer.class || adapter == EditPartViewer.class)
			return getGraphicalViewer();
		else if (adapter == CommandStack.class)
			return getEMFCommandStack();
		else if(adapter == TransactionalEditingDomain.class)
			return getEditingDomain();
		else if (adapter == EditDomain.class)
			return getEditDomain();
		else if (adapter  == IUndoContext.class)
			// used by undo/redo actions to get their undo context
			return undoContext;
		else if (adapter == IPropertySheetPage.class)
			return createPropertySheetPage();
		else if (adapter == IContentOutlinePage.class)
			return getOutlinePage();
		else if (adapter == ZoomManager.class)
			return ((ScalableFreeformRootEditPart)getGraphicalViewer()
			.getRootEditPart()).getZoomManager();
		else if (adapter == UIAwareEditor.class)
				return this;
		
		return super.getAdapter(adapter);
	}
	
	private OverviewOutlinePage getOutlinePage() {
		if(this.outlinePage == null)
			this.outlinePage = new OverviewOutlinePage(this, getGraphicalViewer());
		
		return outlinePage;
	}
	
	private OverviewOutlinePage outlinePage;
	
	public ISelectionListener getSelectionListener() {
		return selectionListener;
	}
	
	public void setSelection(ISelection selection) {
		editorSelection = selection;

		// Update actions
		updateActions();
		for (ISelectionChangedListener listener : selectionChangedListeners) {
			listener.selectionChanged(new SelectionChangedEvent(this, selection));
		}
//		setStatusLineManager(selection);
	}
	
	public void addSelectionChangedListener(ISelectionChangedListener listener) {
		selectionChangedListeners.add(listener);
	}

	public ISelection getSelection() {
		return editorSelection;
	}

	public void removeSelectionChangedListener(	ISelectionChangedListener listener) {
		selectionChangedListeners.remove(listener);
	}

	@Override
	public boolean isDirty() {
		// We track the last operation executed before save was performed
		IUndoableOperation op = getOperationHistory().getUndoOperation(getUndoContext());
		return op != savedOperation;
	}

	@Override
	public boolean isSaveAsAllowed() {
		return false;
	}
	
	/**
	 * Get's the UI Extension for the given object, optionally creates one if it
	 * doesn't exist yet.
	 * @param obj
	 * @param create
	 * @return
	 */
	public PmlUiExtension getUIExtension(final EObject obj, boolean create) {
 
		final PmlUiExtensionMap map = getUIExtensionMap();
		PmlUiExtension ext = map.getExtension().get(obj);
		
		// No extension found...
		if (ext == null && create) {
			// ... create one
			CreateUIExtensionCommmand cmd = new CreateUIExtensionCommmand(getEditingDomain(),map,obj);
			getEMFCommandStack().execute(cmd);
			ext = cmd.getResult().toArray(new PmlUiExtension[1])[0];
			
		}

		return ext;
	}
	
	/**
	 * Returns the EMF CommandStack
	 * @return
	 */
	public EMF2GEFCommandStack getEMFCommandStack(){
		return (EMF2GEFCommandStack)getEditingDomain().getCommandStack();
	}
	
	public PmlUiExtensionMap getUIExtensionMap(){
		return (PmlUiExtensionMap)getUIExtensionResource().getContents().get(0);
	}
	
	public Resource getUIExtensionResource(){
		return uiExtensionResource;
		
	}
	
	protected void setUIExtensionResource(Resource extRes){
		this.uiExtensionResource = extRes;
	}
	
	

	@Override
	public void createPartControl(Composite parent) {
		ToolbarComposite tool = new ToolbarComposite(this,parent, SWT.None);
		this.graphicalViewer = createGraphicalViewer(tool);
		this.graphicalViewer.getControl().setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
//		g = new Graph(parent, SWT.None);
//		TaskGraph graph = getGraph();
//		Map<TaskGroup, GraphNode> mapping = new HashMap<TaskGroup, GraphNode>();
//		for(TaskGroup group : graph.getGroups()){
//			GraphContainer container = new GraphContainer(g, SWT.NONE, group.getName());
//			mapping.put(group, container);
//			for(Task t : group.getTasks()){
//				new GraphNode(container,SWT.NONE,t.getName());
//			}
//			container.open(false);
//			container.setLayoutAlgorithm(new HorizontalLayoutAlgorithm(LayoutStyles.NO_LAYOUT_NODE_RESIZING), true);
//		}
//		
//		for(Edge e : graph.getEdges()){
//			new GraphConnection(g,SWT.NONE,mapping.get(e.getFrom()),mapping.get(e.getTo()));
//		}
//		g.setLayoutAlgorithm(new CompositeLayoutAlgorithm(LayoutStyles.NO_LAYOUT_NODE_RESIZING, 
//				new LayoutAlgorithm[] { 
//				new TreeLayoutAlgorithm(LayoutStyles.NO_LAYOUT_NODE_RESIZING), 
//				new HorizontalShift(LayoutStyles.NO_LAYOUT_NODE_RESIZING) }), true);
//		g.setLayoutAlgorithm(new TreeLayoutAlgorithm(),	true);
	}

	@Override
	public void setFocus() {
		getGraphicalViewer().getControl().setFocus();
//		g.setFocus();
		
	}
	
	public GraphicalViewer getGraphicalViewer() {
		return graphicalViewer;
	}
	
	/**
	 * Creates a new GraphicalViewer, configures, registers and initializes it.
	 * 
	 * @param parent
	 * @return
	 */
	private GraphicalViewer createGraphicalViewer(Composite parent) {
		// Create the viewer
		GraphicalViewer viewer = new ScrollingGraphicalViewer();
		viewer.createControl(parent);
		
		// Configure the viewer
		viewer.getControl().setBackground(ColorConstants.listBackground);
		ScalableFreeformRootEditPart root = new ScalableFreeformRootEditPart();
		viewer.setRootEditPart(root);
		
		ArrayList<String> zoomLevels = new ArrayList<String>(3);
		zoomLevels.add(ZoomManager.FIT_ALL);
		zoomLevels.add(ZoomManager.FIT_WIDTH);
		zoomLevels.add(ZoomManager.FIT_HEIGHT);
		root.getZoomManager().setZoomLevelContributions(zoomLevels);
		
		// Create zoom actions
		IAction zoomIn = new ZoomInAction(root.getZoomManager());
		IAction zoomOut = new ZoomOutAction(root.getZoomManager());
		

		// also bind the actions to keyboard shortcuts
//		getSite().getKeyBindingService().registerAction(zoomIn);
//		getSite().getKeyBindingService().registerAction(zoomOut);
		
		IWorkbenchWindow window = getSite().getWorkbenchWindow();
		IHandlerService handlerService = (IHandlerService)
		window.getService(IHandlerService.class);
		
		// TODO: replace with undeprecated methods
		handlerService.activateHandler(zoomIn.getId(),new ActionHandler(zoomIn));
		handlerService.activateHandler(zoomOut.getId(),new ActionHandler(zoomOut));
		
		// Hook the viewer into the EditDomain
		getEditDomain().addViewer(viewer);
		
		// Activate the viewer as selection provider for Eclipse
		getSite().setSelectionProvider(viewer);
		
		ContextMenuProvider contextProvider = new RolesetDiagramContextMenuProvider(viewer, getActionRegistry());
		viewer.setContextMenu(contextProvider);
		
		// Register the contextMenu provider
		getSite().registerContextMenu(PlanDesignerConstants.ROLESET_EDITOR_ID, 
				new RolesetDiagramContextMenuProvider(viewer,getActionRegistry()), 
				viewer);
		
		

		// Initializes the viewer with input
		viewer.setEditPartFactory(getEditPartFactory());
		
		if(getTaskGraph() != null){
			viewer.setContents(getTaskGraph());
		}
		
		// Register DropTarget listener
		viewer.addDropTargetListener(new DiagramDropTargetListener(viewer));

		return viewer;
	}
	
	protected EditPartFactory getEditPartFactory() {
		return new RolesetEditPartFactory();
	}
	
	/**
	 * Returns the inner GEF commandStack of the TransactionalEditingDomain
	 * @return
	 */
	public CommandStack getGEFCommandStack() {
		return ((EMF2GEFCommandStack)getEditingDomain().getCommandStack()).getCommandStack4GEF();
	}
	
	public PMLTransactionalEditingDomain getEditingDomain() {
		if(editingDomain == null){
			adapterFactory = new ComposedAdapterFactory(ComposedAdapterFactory.Descriptor.Registry.INSTANCE);

			adapterFactory.addAdapterFactory(new ResourceItemProviderAdapterFactory());
			adapterFactory.addAdapterFactory(new AlicaItemProviderAdapterFactory());
			adapterFactory.addAdapterFactory(new ReflectiveItemProviderAdapterFactory());
			
			// Connect to the shared editing domain
			editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
			

		}
		return editingDomain;
	}
	
	public EditDomain getEditDomain() {
		if (this.editDomain == null){
			this.editDomain = new DefaultEditDomain(this){
				@Override
				public CommandStack getCommandStack() {
					return RolesetEditor.this.getGEFCommandStack();
				}
			};
//			this.editDomain.setPaletteRoot(getPaletteRoot());
		}

		return editDomain;
	}

	public Resource getRolesetResource() {
		return rolesetResource;
	}

	private void setRolesetResource(Resource rolesetResource) {
		this.rolesetResource = rolesetResource;
	}
	
	public TaskGraph getTaskGraph() {
		return graph;
	}

	public void setGraph(TaskGraph graph) {
		this.graph = graph;
	}
	
	protected ObjectUndoContext getUndoContext() {
		return undoContext;
	}
	
	/**
	 * Returns the undoable <code>PropertySheetPage</code> for this editor. We cannot
	 * cache this instance but we will always keep a reference to the current page
	 * for refreshing purposes.
	 * 
	 * @return the undoable <code>PropertySheetPage</code>
	 */
	protected TabbedPropertySheetPage createPropertySheetPage() {
		propertyPage =  new PMLTabbedPropertySheetPage(this);
		return propertyPage;
	}

	public AdapterFactory getAdapterFactory() {
		return adapterFactory;
	}

	public String getContributorId() {
		return PlanDesignerConstants.PLAN_EDITOR_ID;
	}
	
	/**
	 * Lazily creates and returns the action registry.
	 * @return the action registry
	 */
	protected ActionRegistry getActionRegistry() {
		if (actionRegistry == null)
			actionRegistry = new ActionRegistry();
		return actionRegistry;
	}
	
	private Resource getTaskGraphResource(){
		return getTaskGraph().eResource();
	}

	public GraphicalEditPart getRootEditPart()
	{
		return (GraphicalEditPart)getGraphicalViewer().getEditPartRegistry().get(getRoleSet());
	}

}
