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

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.eclipse.core.commands.operations.IOperationHistory;
import org.eclipse.core.commands.operations.IOperationHistoryListener;
import org.eclipse.core.commands.operations.IUndoContext;
import org.eclipse.core.commands.operations.IUndoableOperation;
import org.eclipse.core.commands.operations.ObjectUndoContext;
import org.eclipse.core.commands.operations.OperationHistoryEvent;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IWorkspace;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.IPath;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.MultiStatus;
import org.eclipse.core.runtime.Status;
import org.eclipse.draw2d.ColorConstants;
import org.eclipse.emf.common.notify.AdapterFactory;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature.Setting;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.edit.domain.IEditingDomainProvider;
import org.eclipse.emf.edit.provider.ComposedAdapterFactory;
import org.eclipse.emf.edit.provider.ReflectiveItemProviderAdapterFactory;
import org.eclipse.emf.edit.provider.resource.ResourceItemProviderAdapterFactory;
import org.eclipse.emf.edit.ui.action.EditingDomainActionBarContributor;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.RollbackException;
import org.eclipse.emf.transaction.Transaction;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.emf.workspace.IWorkspaceCommandStack;
import org.eclipse.emf.workspace.ResourceUndoContext;
import org.eclipse.emf.workspace.util.WorkspaceSynchronizer;
import org.eclipse.gef.DefaultEditDomain;
import org.eclipse.gef.EditDomain;
import org.eclipse.gef.EditPartFactory;
import org.eclipse.gef.EditPartViewer;
import org.eclipse.gef.GraphicalEditPart;
import org.eclipse.gef.GraphicalViewer;
import org.eclipse.gef.commands.CommandStack;
import org.eclipse.gef.editparts.ScalableFreeformRootEditPart;
import org.eclipse.gef.editparts.ZoomManager;
import org.eclipse.gef.palette.PaletteRoot;
import org.eclipse.gef.ui.actions.ZoomInAction;
import org.eclipse.gef.ui.actions.ZoomOutAction;
import org.eclipse.gef.ui.palette.FlyoutPaletteComposite;
import org.eclipse.gef.ui.palette.FlyoutPaletteComposite.FlyoutPreferences;
import org.eclipse.gef.ui.palette.PaletteViewer;
import org.eclipse.gef.ui.palette.PaletteViewerProvider;
import org.eclipse.gef.ui.parts.ScrollingGraphicalViewer;
import org.eclipse.jface.action.GroupMarker;
import org.eclipse.jface.action.IAction;
import org.eclipse.jface.action.MenuManager;
import org.eclipse.jface.dialogs.ErrorDialog;
import org.eclipse.jface.dialogs.ProgressMonitorDialog;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.ISelectionProvider;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.ui.IActionBars;
import org.eclipse.ui.IEditorInput;
import org.eclipse.ui.IEditorPart;
import org.eclipse.ui.IEditorSite;
import org.eclipse.ui.IFileEditorInput;
import org.eclipse.ui.IPartListener;
import org.eclipse.ui.ISelectionListener;
import org.eclipse.ui.IWorkbenchActionConstants;
import org.eclipse.ui.IWorkbenchPart;
import org.eclipse.ui.IWorkbenchPartSite;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.actions.WorkspaceModifyOperation;
import org.eclipse.ui.dialogs.SaveAsDialog;
import org.eclipse.ui.part.EditorPart;
import org.eclipse.ui.part.FileEditorInput;
import org.eclipse.ui.views.contentoutline.ContentOutline;
import org.eclipse.ui.views.contentoutline.IContentOutlinePage;
import org.eclipse.ui.views.properties.IPropertySheetPage;
import org.eclipse.ui.views.properties.PropertySheet;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.provider.AlicaItemProviderAdapterFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.provider.PmlUIExtensionModelItemProviderAdapterFactory;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanEditorEditPartFactory;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.HiddenElementProviderFactory;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.PlanEditorHiddenElementProviderFactory;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CalculatePlanCardinalitiesCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CreateUIExtensionCommmand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.EMF2GEFCommandStack;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.EnsurePlanParametrisationConsistencyCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.ICommandStackTabbedPropertySheetPageContributor;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.PMLTabbedPropertySheetPage;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.DiagramDropTargetListener;
import de.uni_kassel.vs.cn.planDesigner.ui.util.OverviewOutlinePage;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanMapper;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanmodellerEditorPaletteFactory;
import de.uni_kassel.vs.cn.planDesigner.ui.util.UIAwareEditor;

public class PlanEditor extends EditorPart implements ICommandStackTabbedPropertySheetPageContributor, ISelectionProvider, IEditingDomainProvider, UIAwareEditor{

	/**
	 * The EditDomain which bundles the GraphicalViewer, the PaletteViewer
	 * the commandStack. The commandStack will be a EMF2GEFCommandStack.GEFCommandStack
	 * to work with the EMF Editing domain.
	 * 
	 */
	private EditDomain editDomain;
	
	private WorkspaceSynchronizer workspaceSynchronizer;
	
	/**
	 * The editingDomain is the EMF side. It has the connection to the model and also
	 * a commandStack which will be a EMF2GEFCommandStack.
	 */
	private PMLTransactionalEditingDomain editingDomain;
	
	/**
	 * The GraphicalViewer
	 */
	private GraphicalViewer graphicalViewer;

	/**
	 * The paletteViewer
	 */
	private PaletteViewer paletteViewer;
	
	private FlyoutPaletteComposite flyoutPaletteComposite;
	
	private PaletteViewerProvider provider;
	
	// We track dirty state by the last operation executed when saved
	private IUndoableOperation savedOperation;

	/**
	 * The paletteRoot
	 */
	private PaletteRoot paletteRoot;
	
	private OverviewOutlinePage outlinePage;
	
	private PMLTabbedPropertySheetPage propertyPage;
	
	private PlanMapper planMapper;
	
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
//				if (affectedResources.contains(getPlanMapper().getPlanResource()) || 
//						affectedResources.contains(getUIExtensionResource())) {
//WARNING: THIS IS HIGHLY EXPERIMENTAL!				
				if (PlanEditorUtils.isModifiedResourceAffected(PlanEditor.this, affectedResources)) {
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
				if (affectedResources.contains(getPlanMapper().getPlanResource()) || 
						affectedResources.contains(getUIExtensionResource())) {
//				if (PlanEditorUtils.isModifiedResourceAffected(PlanEditor.this, affectedResources)) {
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
	
	private IPartListener partListener = new IPartListener() {
		// If an open, unsaved file was deleted, query the user to either do a "Save As"
		// or close the editor.
		public void partActivated(IWorkbenchPart part) {
			if (part instanceof ContentOutline) {
				if (((ContentOutline)part).getCurrentPage() == getOutlinePage()) {
					getActionBarContributor().setActiveEditor(PlanEditor.this);
				}
			} else if (part instanceof PropertySheet) {
				if (((PropertySheet)part).getCurrentPage() == propertyPage) {
					getActionBarContributor().setActiveEditor(PlanEditor.this);
					handleActivate();
				}
			}
			
			else if (part == PlanEditor.this) {
				handleActivate();
			}
		}
		public void partBroughtToTop(IWorkbenchPart part) {}
		public void partClosed(IWorkbenchPart part) {}
		public void partDeactivated(IWorkbenchPart part) {}
		public void partOpened(IWorkbenchPart part) {}
	};

	/**
	 * The selectionListener.
	 */
	private ISelectionListener selectionListener = new ISelectionListener() {
		public void selectionChanged(IWorkbenchPart part, ISelection selection) {
			setSelection(selection);
		}
	};

	private Resource uiExtensionResource;
	
	private ObjectUndoContext undoContext;
	
	/**
	 * This keeps track of all the {@link org.eclipse.jface.viewers.ISelectionChangedListener}s that are listening to this editor.
	 */
	protected Collection<ISelectionChangedListener> selectionChangedListeners = new ArrayList<ISelectionChangedListener>();

	/**
	 * This keeps track of the selection of the editor as a whole.
	 */
	protected ISelection editorSelection = StructuredSelection.EMPTY;

	/**
	 * Resources that have been saved.
	 */
	protected Collection<Resource> savedResources = new ArrayList<Resource>();

	/**
	 * This is the one adapter factory used for providing views of the model.
	 */
	protected ComposedAdapterFactory adapterFactory;

	/**
	 * Resources that have been changed since last activation.
	 */
	protected Collection<Resource> changedResources = new ArrayList<Resource>();

	/**
	 * Resources that have been removed since last activation.
	 */
	protected Collection<Resource> removedResources = new ArrayList<Resource>();
	
	/**
	 * Resources that have been moved since last activation.
	 */
	//.CUSTOM: Demonstrates the WorkspaceSynchronizer's handling of moves
	protected Map<Resource, URI> movedResources = new HashMap<Resource, URI>();

	private HiddenElementProviderFactory hiddenElementProvider;
	
	public PlanEditor() {
		super();
		
		undoContext = new ObjectUndoContext(this, PlanDesignerConstants.PLAN_EDITOR_ID);
	}
	
	private IOperationHistory getOperationHistory() {
		return ((IWorkspaceCommandStack) getEditingDomain().getCommandStack()).getOperationHistory();
	}

	@Override
	public void doSave(IProgressMonitor monitor) {
		WorkspaceModifyOperation operation =
			new WorkspaceModifyOperation() {
			// This is the method that gets invoked when the operation runs.
			public void execute(IProgressMonitor monitor) {
//				// Calculate the min/max cardinalities
//				new RecordingCommand(getEditingDomain()){
//					@Override
//					protected void doExecute() {
//						getPlan().calculateCardinalities();
//					}
//				}.execute();
				getEditingDomain().getCommandStack().execute(new CalculatePlanCardinalitiesCommand(getPlan()));
				getEditingDomain().getCommandStack().execute(new EnsurePlanParametrisationConsistencyCommand(getPlan()));
				getEditingDomain().getCommandStack().execute(new RecordingCommand(getEditingDomain()){
					@Override
					protected void doExecute() {
						try {
							// Let the planMapper save the plan
							getPlanMapper().save();
							
							// Remove any corrupt extensions
							removeCorruptExtensions();
							// Save the UI Extension resource
							getUIExtensionResource().save(null);
							
							Resource taskrepoResource = CommonUtils.getTaskRepository(editingDomain, true).eResource();
							// Save the task repository
							taskrepoResource.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
							
							List<Resource> behaviours = PlanEditorUtils.collectModifiedResources(PlanEditor.this, false);
							for(Resource r : behaviours)
							{
								r.save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
							}
							
							savedResources.addAll(behaviours);
							savedResources.add(getPlanMapper().getPlanResource());
							savedResources.add(getUIExtensionResource());
							savedResources.add(taskrepoResource);
						} catch (IOException e) {
							e.printStackTrace();
						} catch (RollbackException e) {
							e.printStackTrace();
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}
				});
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
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	

	@Override
	public void doSaveAs() {
		performSaveAs();
	}

	@Override
	public void init(IEditorSite site, IEditorInput input) throws PartInitException {
		
		IFile file = ((IFileEditorInput)input).getFile();
		
		if(file == null)
			throw new PartInitException("File for " +input +" not found");
		
		planMapper = new PlanMapper(file);
		// Load the file with the editingDomain
		Resource resource = planMapper.load();
		
		if(resource != null){
			// TODO: We should not need that anymore due to ltk delete support
			checkForUnresolvableReferences(resource, true);
		} else
			throw new PartInitException("Could not load the plan");
		
		// load the ui extension resource
		Resource uiExt = getEditingDomain().loadExtensionResource(resource);
		setUIExtensionResource(uiExt);
		
		// TODO: We should not need that anymore due to ltk delete support
		checkForUnresolvableReferences(uiExt, false);
		
		
		// Store site and Input
		setSite(site);
		setInput(input);
		site.getPage().addPartListener(partListener);
		
		setPartName(file.getName());

		// Add selection change listener
		getSite().getWorkbenchWindow().getSelectionService()
				.addSelectionListener(getSelectionListener());

		
		if(getGraphicalViewer() != null){
			getGraphicalViewer().setContents(getResource());
		}
		
		workspaceSynchronizer = new WorkspaceSynchronizer( getEditingDomain(), createSynchronizationDelegate());
		getOperationHistory().addOperationHistoryListener(historyListener);
		
//		getEditingDomain().addResourceSetListener(getResourceSetListener());
	}
	
	private void checkForUnresolvableReferences(Resource r, boolean displayMsg){
		Map<EObject, Collection<Setting>> proxies = PlanEditorUtils.checkForUnresolvableReferences(this,r);
		
		if(!proxies.isEmpty() && displayMsg){
			// Display a message to the user that some object could not be resolved
			final MultiStatus status = new MultiStatus(
					PlanDesignerActivator.PLUGIN_ID, 
					94, 
					"Some referenced elements could not be found.", 
					null);
			
			for(EObject e : proxies.keySet()){
				status.add(new Status(IStatus.WARNING, PlanDesignerActivator.PLUGIN_ID, 
						"Element " +e +" was removed because it could not be loaded."));
			}
			
			Display.getCurrent().asyncExec(new Runnable(){
				public void run() {
					ErrorDialog.openError(Display.getCurrent().getActiveShell(), "Problem loading plan", 
							"Some elements which were contained in the plan could not be loaded\n" +
							"and were removed from the plan. See Details for further information", status);
				}
			});
			
		}
	}

	private WorkspaceSynchronizer.Delegate createSynchronizationDelegate() {
		return new WorkspaceSynchronizer.Delegate() {
			public boolean handleResourceDeleted(Resource resource) {
				if ((resource == getResource()) && !isDirty()) {
					// just close now without prompt
					getSite().getShell().getDisplay().asyncExec(new Runnable() {
						public void run() {
							getSite().getPage().closeEditor(PlanEditor.this, false);
							PlanEditor.this.dispose();
						}
					});
				} else {
					removedResources.add(resource);					
				}
				return true;
			}	   


			public boolean handleResourceMoved(Resource resource, URI newURI) {
				movedResources.put(resource, newURI);
				
				return true;
			}

			public boolean handleResourceChanged(Resource resource) {
				if (savedResources.contains(resource)) {
                    savedResources.remove(resource);
                } else {
                    changedResources.add(resource);
                }
				
				return true;
			}

			public void dispose() {
				removedResources.clear();
				changedResources.clear();
				movedResources.clear();
			}
		
		};
	}


	@Override
	public void dispose() {
		if(hiddenElementProvider != null)
		{
			hiddenElementProvider.dispose();
		}
		
		// Remove selection listener
		getSite().getWorkbenchWindow().getSelectionService()
				.removeSelectionListener(getSelectionListener());

		if (getActionBarContributor().getActiveEditor() == this) {
			getActionBarContributor().setActiveEditor(null);
		}
		
		workspaceSynchronizer.dispose();
		
		// Remove the operation history stuff
		getOperationHistory().removeOperationHistoryListener(historyListener);
		getOperationHistory().dispose(getUndoContext(), true, true, true);
		
		// We are the only one who is editing the UIExtensionResource, so remove it from the 
		// shared resourceSet
		getUIExtensionResource().eAdapters().clear();
		getEditingDomain().getResourceSet().getResources().remove(getUIExtensionResource());
		
		// TODO: Remove the partListener if we do support saveAs, cause otherwise there will be
		// no registered partListener @see setSite()!
		getSite().getPage().removePartListener(partListener);
		
		getResource().unload();
		
		adapterFactory.dispose();
		
//		getEditingDomain().removeResourceSetListener(getResourceSetListener());
		
		super.dispose();
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

	@Override
	public void createPartControl(Composite parent) {
		this.flyoutPaletteComposite = new FlyoutPaletteComposite(parent, SWT.NONE, getSite().getPage(),
				getPaletteViewerProvider(), getPalettePreferences());
		
		this.graphicalViewer = createGraphicalViewer(flyoutPaletteComposite);
		this.getSite().setSelectionProvider(this.graphicalViewer);
		
		this.flyoutPaletteComposite.setGraphicalControl(this.graphicalViewer.getControl());
	}

	@Override
	public void setFocus() {
		getGraphicalViewer().getControl().setFocus(); 
	}

	public EditDomain getEditDomain() {
		if (this.editDomain == null){
			this.editDomain = new DefaultEditDomain(this){
				@Override
				public CommandStack getCommandStack() {
					return PlanEditor.this.getGEFCommandStack();
				}
			};
			this.editDomain.setPaletteRoot(getPaletteRoot());
		}

		return editDomain;
	}

	public void setEditDomain(EditDomain editDomain) {
		this.editDomain = editDomain;
		this.editDomain.setPaletteRoot(getPaletteRoot());
	}

	/**
	 * Returns the inner GEF commandStack of the TransactionalEditingDomain
	 * @return
	 */
	public CommandStack getGEFCommandStack() {
		return ((EMF2GEFCommandStack)getEditingDomain().getCommandStack()).getCommandStack4GEF();
	}
	
	/**
	 * Returns the EMF CommandStack
	 * @return
	 */
	public EMF2GEFCommandStack getEMFCommandStack(){
		return (EMF2GEFCommandStack)getEditingDomain().getCommandStack();
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
	@SuppressWarnings("deprecation")
	private GraphicalViewer createGraphicalViewer(Composite parent) {
		// Create the viewer
		GraphicalViewer viewer = new ScrollingGraphicalViewer();
		viewer.createControl(parent);

		// Configure the viewer
		viewer.getControl().setBackground(ColorConstants.listBackground);
		ScalableFreeformRootEditPart root = new ScalableFreeformRootEditPart();
		
		List<String> zoomLevels = new ArrayList<String>(3);
		zoomLevels.add(ZoomManager.FIT_ALL);
		zoomLevels.add(ZoomManager.FIT_WIDTH);
		zoomLevels.add(ZoomManager.FIT_HEIGHT);
		root.getZoomManager().setZoomLevelContributions(zoomLevels);
		
		viewer.setRootEditPart(root);
		
		// Create zoom actions
		IAction zoomIn = new ZoomInAction(root.getZoomManager());
		IAction zoomOut = new ZoomOutAction(root.getZoomManager());
		// also bind the actions to keyboard shortcuts
		getSite().getKeyBindingService().registerAction(zoomIn);
		getSite().getKeyBindingService().registerAction(zoomOut);
		

		// Hook the viewer into the EditDomain
		getEditDomain().addViewer(viewer);

		// Activate the viewer as selection provider for Eclipse
		getSite().setSelectionProvider(viewer);

		// Initializes the viewer with input
		viewer.setEditPartFactory(getEditPartFactory());
		
		if(getResource() != null){
			viewer.setContents(getResource());
		}
		
		// Register DropTarget listener
		viewer.addDropTargetListener(new DiagramDropTargetListener(viewer));
		
		final List<String> allowedIds = new ArrayList<String>();
		allowedIds.add("de.uni_kassel.vs.cn");
		allowedIds.add(IWorkbenchActionConstants.MB_ADDITIONS);
		
		MenuManager menuMgr = new MenuManager();
		// Add the marker where other plugins can contribute new actions
		menuMgr.add(new GroupMarker(IWorkbenchActionConstants.MB_ADDITIONS));
		viewer.setContextMenu(menuMgr);
		
		getSite().registerContextMenu(menuMgr, getSite().getSelectionProvider());

		return viewer;
	}

	protected EditPartFactory getEditPartFactory() {
		return new PlanEditorEditPartFactory();
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
		else if(adapter == HiddenElementProviderFactory.class)
		{
			return getHiddenElementProviderFactory();
		}
			
		

		return super.getAdapter(adapter);
	}
	
	private Object getHiddenElementProviderFactory()
	{
		if(hiddenElementProvider == null)
		{
			hiddenElementProvider = new PlanEditorHiddenElementProviderFactory(this);
		}
		
		return hiddenElementProvider;
	}

	protected PaletteRoot getPaletteRoot() {
		if (this.paletteRoot == null) {
			// Create the paletteRoot
			this.paletteRoot = PlanmodellerEditorPaletteFactory.createPalette();
		}
		return this.paletteRoot;
	}

	public PaletteViewer getPaletteViewer() {
		return paletteViewer;
	}

	public ISelectionListener getSelectionListener() {
		return selectionListener;
	}

	public Resource getUIExtensionResource(){
//		if(uiExtensionResource == null){
//			uiExtensionResource = getEditingDomain().loadExtensionResource(getResource());
//		}
//		
//		return uiExtensionResource;
		
		return uiExtensionResource;
		
	}
	
	protected void setUIExtensionResource(Resource extRes){
		this.uiExtensionResource = extRes;
	}

	public EditingDomainActionBarContributor getActionBarContributor() {
		return (EditingDomainActionBarContributor)getEditorSite().getActionBarContributor();
	}
	
	public IActionBars getActionBars() {
		return getActionBarContributor().getActionBars();
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

//	public EList<EObject> getPlanDiagram() {
//		return planDiagram;
//	}

	/**
	 * Returns the palette viewer provider that is used to create palettes for the view and
	 * the flyout.  Creates one if it doesn't already exist.
	 * 
	 * @return	the PaletteViewerProvider that can be used to create PaletteViewers for
	 * 			this editor
	 * @see	#createPaletteViewerProvider()
	 */
	protected final PaletteViewerProvider getPaletteViewerProvider() {
		if (provider == null)
			provider = createPaletteViewerProvider();
		
		return provider;
	}
	
	/**
	 * Creates a PaletteViewerProvider that will be used to create palettes for the view
	 * and the flyout.
	 * 
	 * @return	the palette provider
	 */
	protected PaletteViewerProvider createPaletteViewerProvider() {
		return new PaletteViewerProvider(getEditDomain());
	}
	
	/**
	 * This method returns a FlyoutPreferences object that stores the flyout
	 * settings in the Planmodeller plugin. 
	 * @return	the FlyoutPreferences object used to save the flyout palette's preferences 
	 */
	protected FlyoutPreferences getPalettePreferences() {
		// TODO: The FlyoutPaletteComposite seems to be to old to switch to the following code
		//IPreferencesService preferencesService = Platform.getPreferencesService();
		//return FlyoutPaletteComposite.createFlyoutPreferences(preferencesService.getRootNode());
		
		return FlyoutPaletteComposite.createFlyoutPreferences(PlanDesignerActivator.getDefault().getPluginPreferences());
	}

	public Plan getPlan() {
//		if(this.plan == null){
//			this.plan = (Plan)getResource().getContents().get(0);
//		}
		return getPlanMapper().getPlan();
	}

	public Resource getResource() {
		return getPlanMapper().getPlanResource();
	}

	private OverviewOutlinePage getOutlinePage() {
		if(this.outlinePage == null)
			this.outlinePage = new OverviewOutlinePage(this, getGraphicalViewer());
		
		return outlinePage;
	}

	protected void closeEditor(boolean save) {
		getSite().getPage().closeEditor(PlanEditor.this, save);
	}
	
	protected boolean performSaveAs() {
		SaveAsDialog dialog = new SaveAsDialog(getSite().getWorkbenchWindow().getShell());
		dialog.setOriginalFile(((IFileEditorInput)getEditorInput()).getFile());
		dialog.open();
		IPath path= dialog.getResult();
		
		if (path == null)
			return false;
		
		IWorkspace workspace = ResourcesPlugin.getWorkspace();
		final IFile file= workspace.getRoot().getFile(path);
		
		if (!file.exists()) {
			WorkspaceModifyOperation op= new WorkspaceModifyOperation() {
				public void execute(final IProgressMonitor monitor) {
					// TODO: Perform save as here...
					
//					saveProperties();
//					try {
//						ByteArrayOutputStream out = new ByteArrayOutputStream();
//						writeToOutputStream(out);
//						file.create(new ByteArrayInputStream(out.toByteArray()), true, monitor);
//						out.close();
//					} 
//					catch (Exception e) {
//						e.printStackTrace();
//					}
				}
			};
			try {
				new ProgressMonitorDialog(getSite().getWorkbenchWindow().getShell())
						.run(false, true, op);			
			}
			catch (Exception e) {
				e.printStackTrace();
			}
		}
		
		try {
			setInput(new FileEditorInput(file));
			getEMFCommandStack().saveIsDone();
			
		} 
		catch (Exception e) {
			e.printStackTrace();
		} 
		return true;
	}
	
	@Override
	protected void setSite(IWorkbenchPartSite site) {
		super.setSite(site);
		
		// TODO: Add the partListener if we do support saveAs!
//		getSite().getWorkbenchWindow().getPartService().addPartListener(partListener);
	}

	public String getContributorId() {
		return PlanDesignerConstants.PLAN_EDITOR_ID;
	}

	public PMLTransactionalEditingDomain getEditingDomain() {
		if(editingDomain == null){
			adapterFactory = new ComposedAdapterFactory(ComposedAdapterFactory.Descriptor.Registry.INSTANCE);

			adapterFactory.addAdapterFactory(new ResourceItemProviderAdapterFactory());
			adapterFactory.addAdapterFactory(new AlicaItemProviderAdapterFactory());
			adapterFactory.addAdapterFactory(new ReflectiveItemProviderAdapterFactory());
			
			// TODO: Thats not working, i.e. the domain doenst find the provider while searching
			// form them in creating new commands
			adapterFactory.addAdapterFactory(new PmlUIExtensionModelItemProviderAdapterFactory());
			
			editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
			

		}
		return editingDomain;
	}

	protected ObjectUndoContext getUndoContext() {
		return undoContext;
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

	public void setSelection(ISelection selection) {
		editorSelection = selection;

		for (ISelectionChangedListener listener : selectionChangedListeners) {
			listener.selectionChanged(new SelectionChangedEvent(this, selection));
		}
//		setStatusLineManager(selection);
	}
	
	public PmlUiExtensionMap getUIExtensionMap(){
//		if(uiExtensionMap == null){
//			uiExtensionMap = (PmlUiExtensionMap)getUIExtensionResource().getContents().get(0);
//		}
//		return uiExtensionMap;
		return (PmlUiExtensionMap)getUIExtensionResource().getContents().get(0);
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

	public AdapterFactory getAdapterFactory() {
		return adapterFactory;
	}

	/**
	 * Handles what to do with changed resource on activation.
	 */
	protected void handleChangedResource() {
		Resource res = getResource();
		
		if (PlanEditorUtils.isModifiedResourceAffected(this, changedResources) && handleDirtyConflict()) {
			changedResources.removeAll(PlanEditorUtils.collectModifiedResources(this,true));
			changedResources.remove(getPlanMapper().getPlanResource());
			
			getOperationHistory().dispose(undoContext, true, true, true);
			firePropertyChange(IEditorPart.PROP_DIRTY);

			if (res.isLoaded()) {
				res.unload();
				getUIExtensionResource().unload();
				try {
					res.load(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
					getUIExtensionResource().load(null);
				} catch (IOException exception) {
					exception.printStackTrace();
				}
			}

			// Refresh the graphicalViewer
			getGraphicalViewer().setContents(res);
		}
	}
	
	/**
	 * Shows a dialog that asks if conflicting changes should be discarded.
	 * <!-- begin-user-doc -->
	 * <!-- end-user-doc -->
	 * @generated
	 */
	protected boolean handleDirtyConflict() {
//		return
//			MessageDialog.openQuestion
//				(getSite().getShell(),
//				 "File conflict", //$NON-NLS-1$
//				 "Plan Designer has encountered a conflict with this file due to changes from " +
//				 "externally. \n\nDo you want to discard all your changes and reload your file?"); //$NON-NLS-1$
		return false;
	}

	/**
	 * Handles activation of the editor or it's associated views.
	 */
	protected void handleActivate() {
		// Refresh any actions that may become enabled or disabled.
		setSelection(getSelection());

		try {
			final Resource res = getResource();
			
			if (PlanEditorUtils.isModifiedResourceAffected(this, removedResources)) {
				// Something has been removed, first check if the acutal pml file
				// was removed. If so we ask the user how to proceed
				if(removedResources.contains(getResource()) && handleDirtyConflict())
					getSite().getPage().closeEditor(PlanEditor.this, false);
				else{
					// Something else has been removed
				}
					
			} else if (movedResources.containsKey(res)) {
				if (savedResources.contains(res)) {
					getOperationHistory().dispose(undoContext, true, true, true);
					
					// change saved resource's URI and remove from map
					res.setURI(movedResources.remove(res));
						
					// must change my editor input
					IEditorInput newInput = new FileEditorInput(
							WorkspaceSynchronizer.getFile(res));
					setInputWithNotify(newInput);
					setPartName(newInput.getName());
				} else {
//					handleMovedResource();
					System.err.println("Resource moved!!");
				}
			} else if (PlanEditorUtils.isModifiedResourceAffected(this, changedResources)) {
				changedResources.removeAll(savedResources);
				handleChangedResource();
			}
		} finally {
			removedResources.clear();
			changedResources.clear();
			movedResources.clear();
			savedResources.clear();
		}
	}
	
	/**
	 * Handles what to do with moved resource on activation.
	 */
	protected void handleMovedResource() {
		if (!isDirty() || handleDirtyConflict()) {
			Resource res = getResource();
			URI newURI = movedResources.get(res);
			
			if (newURI != null) {
				if (res.isLoaded()) {
					// unload
					res.unload();
				}
	
				// load the new URI in another editor
				res.getResourceSet().getResource(newURI, true);
			}
		}
	}

	public PlanMapper getPlanMapper() {
		return planMapper;
	}

	public GraphicalEditPart getRootEditPart()
	{
		return (GraphicalEditPart)getGraphicalViewer().getEditPartRegistry().get(getPlan());		
	}
}
