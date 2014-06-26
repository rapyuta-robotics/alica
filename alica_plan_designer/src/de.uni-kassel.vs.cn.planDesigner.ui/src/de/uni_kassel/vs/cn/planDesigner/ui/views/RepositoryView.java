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
package de.uni_kassel.vs.cn.planDesigner.ui.views;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IResourceChangeEvent;
import org.eclipse.core.resources.IResourceChangeListener;
import org.eclipse.core.resources.IWorkspace;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.emf.common.notify.AdapterFactory;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.edit.provider.ComposedAdapterFactory;
import org.eclipse.emf.transaction.RunnableWithResult;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.emf.workspace.util.WorkspaceSynchronizer;
import org.eclipse.gef.dnd.TemplateTransfer;
import org.eclipse.jface.action.GroupMarker;
import org.eclipse.jface.action.MenuManager;
import org.eclipse.jface.resource.ImageRegistry;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.ILabelProvider;
import org.eclipse.jface.viewers.ILabelProviderListener;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.StructuredViewer;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.viewers.ViewerSorter;
import org.eclipse.jface.wizard.WizardDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.dnd.DND;
import org.eclipse.swt.dnd.DragSource;
import org.eclipse.swt.dnd.DragSourceEvent;
import org.eclipse.swt.dnd.DragSourceListener;
import org.eclipse.swt.dnd.Transfer;
import org.eclipse.swt.events.FocusAdapter;
import org.eclipse.swt.events.FocusEvent;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Menu;
import org.eclipse.swt.widgets.TabFolder;
import org.eclipse.swt.widgets.TabItem;
import org.eclipse.ui.IActionBars;
import org.eclipse.ui.ISharedImages;
import org.eclipse.ui.IWorkbenchActionConstants;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.actions.ActionFactory;
import org.eclipse.ui.ide.IDE;
import org.eclipse.ui.part.Page;
import org.eclipse.ui.part.ViewPart;
import org.eclipse.ui.views.properties.IPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;
import de.uni_kassel.vs.cn.planDesigner.alica.Task;
import de.uni_kassel.vs.cn.planDesigner.alica.TaskRepository;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.actions.DeleteBehaviourConfigurationAction;
import de.uni_kassel.vs.cn.planDesigner.ui.actions.DeletePlanFromRepositoryAction;
import de.uni_kassel.vs.cn.planDesigner.ui.actions.DeletePlanTypesFromRepositryAction;
import de.uni_kassel.vs.cn.planDesigner.ui.actions.DeleteTaskFromRepositoryAction;
import de.uni_kassel.vs.cn.planDesigner.ui.adapter.MultiObjectNotificationAdapter;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.EMF2GEFCommandStack;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.ICommandStackTabbedPropertySheetPageContributor;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.PMLTabbedPropertySheetPage;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.SelectionProviderMediator;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLPlanTypeConfigurationWizard;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLPlanningProblemConfigurationWizard;

/**
 * The repository view shows 4 different looks at the workspace:
 * <ul>
 * <li>TableViewer showing only Plans</li>
 * <li>TableViewer showing only PlanTypes</li>
 * <li>TableViewer showing only BehaviourConfigurations</li>
 * <li>TableViewer showing the Taskrepository</li>
 * </ul>
 * Each ExpandItem is draggable, the Transfer is a TemplateTransfer with it's template
 * set to the dragged IFile. 
 * @author Zenobios
 *
 */
public class RepositoryView extends ViewPart implements ICommandStackTabbedPropertySheetPageContributor{
	
	private EMF2GEFCommandStack commandStack;
	
	private TabFolder folder;

	private StructuredViewer[] viewers;

	private IResourceChangeListener workspaceChangeListener;
	
	private MultiObjectNotificationAdapter behaviourNotificationAdapter;
	private MultiObjectNotificationAdapter planNotificationAdapter;
	private MultiObjectNotificationAdapter planTypeNotificationAdapter;
	private MultiObjectNotificationAdapter taskNotificationAdapter;
	private MultiObjectNotificationAdapter planningProblemNotificationAdapter;
	
	private PMLTransactionalEditingDomain editingDomain;
	
	/**
	 * The TableViewer for the plans
	 */
	private TableViewer plansViewer;
	
	/**
	 * The TableViewer for the planTypes
	 */
	private TableViewer planTypesViewer;
	
	/**
	 * The TreeViewer for the behaviours
	 */
	private TreeViewer behaviourViewer;
	
	/**
	 * The TreeViewer for the taskRepository
	 */
	private TableViewer taskrepositoryViewer;
	
	/**
	 * The TreeViewer for the planningProblems
	 */
	private TableViewer planningProblemsViewer;
	
	/**
	 * This is the one adapter factory used for providing views of the model.
	 */
	protected ComposedAdapterFactory adapterFactory;
	
	private class PlansSorter extends ViewerSorter
	{
		private static final int MASTER_PLAN_CATEGORY = 0;
		private static final int OTHER_CATEGORY = 1;
		
		@Override
		public boolean isSorterProperty(Object element, String property) {
			return true;
		}
		
		@Override
		public int category(Object element) {
			if(element instanceof Plan)
			{
				if(((Plan)element).isMasterPlan())
				{
					return MASTER_PLAN_CATEGORY;
				}
				else
				{
					return OTHER_CATEGORY;
				}
			}
			else
			{
				return OTHER_CATEGORY;
			}
		}
	}

	private class TaskrepositoryLabelProvider extends LabelProvider{
		@Override
		public String getText(Object element) {
			if(element instanceof Task){
				return ((Task)element).getName();
			}else
				return element.toString();
		}
		
		@Override
		public Image getImage(Object element) {
			Image img = null;
			if(element instanceof Task){
				img = PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_TASK_16);
			}else
				img = PlatformUI.getWorkbench().getSharedImages().getImage(ISharedImages.IMG_OBJS_ERROR_TSK);
			
			return img;
		}
	}
	
	private class TaskrepositoryContentProvider implements IStructuredContentProvider{

		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
		}

		public Object[] getElements(Object inputElement) {
			Object[] elements =  new Object[]{"No taskrepository found"};
			// We expect a Taskrepository here
			if(inputElement instanceof IWorkspaceRoot){
				// Check if there is a taskrepository file
				IFile taskrepositoryFile = CommonUtils.getTaskRepositoryFile();
				if(taskrepositoryFile.exists()){
					// If that file exists, we assume the contents
					// to be correct
					TaskRepository repository = CommonUtils.getTaskRepository(getEditingDomain(), false);
					
					MultiObjectNotificationAdapter adapter = getTaskNotificationAdapter();
					for(Task t : repository.getTasks()){
						adapter.addToObject(t);
					}
					elements = repository.getTasks().toArray();
				}
			}
			
			return elements;
		}
	}
	
	private class BehaviourTreeLabelProvider implements ILabelProvider{

		public Image getImage(Object element) {
			if(element instanceof Behaviour)
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_BEHAVIOUR_16);
			else if(element instanceof BehaviourConfiguration)
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_BEHAVIOUR_CONFIGURATION_16);
			else 
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_UNKNOWN_TYPE);
		}

		public String getText(Object element) {
			// This should also be the case
			if(element instanceof PlanElement){
				return (String)PlanEditorUtils.safeLoadFeature(
						getEditingDomain(), 
						(PlanElement)element, 
						AlicaPackage.eINSTANCE.getPlanElement_Name());
			}
			else
				return element.toString();
		}

		public void addListener(ILabelProviderListener listener) {
			
		}

		public void dispose() {
			
		}

		public boolean isLabelProperty(Object element, String property) {
			return false;
		}

		public void removeListener(ILabelProviderListener listener) {
			
		}
		
	}
	
	private class BehaviourTreeContentProvider implements ITreeContentProvider{

		public Object[] getChildren(final Object parentElement) {
			// This should always be the case here
			if(parentElement instanceof Behaviour){
				Object[] result = null;
				try {
					result = (Object[])getEditingDomain().runExclusive(new RunnableWithResult.Impl<Object[]>(){
						public void run() {
							setResult(((Behaviour)parentElement).getConfigurations().toArray());
							
						}
					});
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				return result;
			}
				
			else
				return new Object[0];
		}

		public Object getParent(Object element) {
			if(element instanceof BehaviourConfiguration)
				return PlanEditorUtils.safeLoadFeature(
						getEditingDomain(), 
						(BehaviourConfiguration)element, 
						AlicaPackage.eINSTANCE.getBehaviourConfiguration_Behaviour());
			else
				return null;
		}

		public boolean hasChildren(Object element) {
			if(element instanceof Behaviour && hasConfigurations((Behaviour)element))
				return true;
			else 
				return false;
		}

		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
		}

		public Object[] getElements(Object inputElement) {
			final Set<IFile> behaviourFiles = PlanEditorUtils.collectAllFilesWithExtension("beh");
			
			// Refresh the notification adapter
			getBehaviourNotificationAdapter().removeFromObjects();
			
			// If there aren't any .beh files we have nothing to return
			if(behaviourFiles.isEmpty())
				return new Object[]{"No Behaviours found."};
			
			List<Resource> loaded = new ArrayList<Resource>();
			// Load the behaviour file
			for(IFile behaviourFile : behaviourFiles)
				loaded.add(getEditingDomain().load(behaviourFile));
			
			// Add all loaded resources to the global loadedResources map
//			loadedResources.addAll(loaded);
			
			List<Behaviour> behaviours = new ArrayList<Behaviour>();
			
			// Get the Behaviour of each file
			for(Resource r : loaded){
//				r.unload();
//				EcoreUtil.resolveAll(r);
				behaviours.add((Behaviour)r.getContents().get(0));
			}
			
			// Add all interesting objects to the behaviourNotificationAdapter
			for(Behaviour behaviour : behaviours){
				getBehaviourNotificationAdapter().addToObject(behaviour);
				for(BehaviourConfiguration config : behaviour.getConfigurations()){
					getBehaviourNotificationAdapter().addToObject(config);
				}
			}
				
			return behaviours.toArray();
				
		}
		
		private boolean hasConfigurations(final Behaviour beh){
			boolean result = false;
			try {
				result = (Boolean)getEditingDomain().runExclusive(new RunnableWithResult.Impl<Boolean>(){
					public void run() {
						setResult(beh.getConfigurations().size() > 0);
					}
				});
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return result;
		}
		
	}

	private class ViewContentProvider implements IStructuredContentProvider{
		
		private String fileExtension;
		
		public ViewContentProvider(String fileExtension){
			this.fileExtension = fileExtension;
		}

		public void dispose() {
			// TODO Auto-generated method stub
			
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
			// TODO Auto-generated method stub
//			System.out.println("Input changed: " +newInput);
		}

		public Object[] getElements(Object parent) {
			final Set<IFile> resources = PlanEditorUtils.collectAllFilesWithExtension(fileExtension);
			
			// Find the right notificationAdapter
			MultiObjectNotificationAdapter adapter = null;
			if(fileExtension.equals("pml"))
				adapter = getPlanNotificationAdapter();
			else if( fileExtension.equals("pp"))
				adapter = getPlanningNotificationAdapter();
			else
				adapter = getPlanTypeNotificationAdapter();
			
			// Tell the adapter to remove from all objects
			adapter.removeFromObjects();
			
			if(resources.size() > 0){
				// Load each file
				List<Resource> loaded = new ArrayList<Resource>();
				for(IFile file : resources)
					loaded.add(getEditingDomain().load(file));
				
				// Add all loaded resources to the global loadedResources map
//				loadedResources.addAll(loaded);
				
				EObject[] loadedElements = new EObject[loaded.size()];
				for(int i=0; i < loaded.size(); i++) {
					loadedElements[i] = loaded.get(i).getContents().get(0);
				}
				
				// Add all interesting objects to the notificationAdapter
				for(EObject object : loadedElements){
					adapter.addToObject(object);
				}
				
				return loadedElements;
				
			} else
				return new Object[]{"No elements to display."};
		}
	}

	private class ViewLabelProvider extends LabelProvider {

		public String getText(Object obj) {
			if(obj instanceof PlanElement){
				return (String)PlanEditorUtils.safeLoadFeature(
						getEditingDomain(), 
						(PlanElement)obj, 
						AlicaPackage.eINSTANCE.getPlanElement_Name());
			}
			return obj.toString();
		}

		public Image getImage(Object obj) {
			ImageRegistry reg = PlanDesignerActivator.getDefault().getImageRegistry();
			
			if(obj instanceof Plan)
			{
				if(((Plan)obj).isMasterPlan())
				{
					return reg.get(PlanDesignerConstants.ICON_MASTER_PLAN_16);	
				}
				else
				{
					return reg.get(PlanDesignerConstants.ICON_PLAN_16);	
				}
			}
			else if(obj instanceof PlanType)
			{
				return reg.get(PlanDesignerConstants.ICON_PLANTYPE_16);
			}
			else if(obj instanceof PlanningProblem)
			{
				return reg.get(PlanDesignerConstants.ICON_PLANNING_PROBLEM_16);
			}
			else
			{
				return reg.get(PlanDesignerConstants.ICON_UNKNOWN_TYPE);
			}
		}
	}
	
	/**
	 * The constructor.
	 */
	public RepositoryView() {
		adapterFactory = new ComposedAdapterFactory(ComposedAdapterFactory.Descriptor.Registry.INSTANCE);
		
//		adapterFactory.addAdapterFactory(new ResourceItemProviderAdapterFactory());
//		adapterFactory.addAdapterFactory(new PmlItemProviderAdapterFactory());
//		adapterFactory.addAdapterFactory(new ReflectiveItemProviderAdapterFactory());
	}
	
	private void addDragSupport(final StructuredViewer viewer){
			Control control = viewer.getControl();
		    final DragSource source = new DragSource (control, DND.DROP_MOVE);
		    source.setTransfer(new Transfer[]{TemplateTransfer.getInstance()});
		    source.addDragListener (new DragSourceListener () {
		    	Object draggedElement = null;
		    	
		        public void dragStart(DragSourceEvent event) {
		        	IStructuredSelection selection = (IStructuredSelection)viewer.getSelection();
		            Object firstElement = selection.getFirstElement();
		            
		            // The selection must not be empty and has to be a PlanElement
		            if(firstElement != null && firstElement instanceof PlanElement)
		            {
		            	event.doit = true;
		            	draggedElement = firstElement;
		            }
		            else
		            {
		            	event.doit = false;
		            }
		        }
		        public void dragSetData (DragSourceEvent event) {
		        	if(draggedElement == null)
		        	{
		        		event.doit = false;
		        	}
		        	else
		        	{
		        		event.data = draggedElement;
		        	}
		            
		        }
		        public void dragFinished(DragSourceEvent event) {
//		        	System.out.println("DRAG FINISHED");
		        }
		    });
	}

	/**
	 * This is a callback that will allow us to create the viewer and initialize
	 * it.
	 */
	public void createPartControl(Composite parent) {
		folder = new TabFolder(parent, SWT.TOP);
		folder.setSize(SWT.FILL, SWT.FILL);
		
		// Add the TabItem for the plans
		TabItem tabItem = new TabItem(folder, SWT.NONE);
		tabItem.setText("Plans");
		tabItem.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_PLAN_16));
		
					
		// Create the viewer for the plans
		plansViewer = new TableViewer(folder, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL);
		plansViewer.setContentProvider(new ViewContentProvider("pml"));
		plansViewer.setLabelProvider(new ViewLabelProvider());
		plansViewer.setSorter(new PlansSorter());
		tabItem.setControl(plansViewer.getControl());
		plansViewer.addDoubleClickListener(new IDoubleClickListener(){
			public void doubleClick(DoubleClickEvent event) {
				Object selection = ((IStructuredSelection)event.getSelection()).getFirstElement();
				if(selection instanceof EObject){
					try {
						// TODO: Okay, that's a hack. It is possible
						// that the underlying resource was unloaded,
						// so we can't retrieve the filename because the
						// elements resource is null. So we resolve
						// the element and the refresh the viewer.
						// This mechanism should be replaced in the future!
						EObject element = (EObject)selection;
						if(element.eIsProxy()){
							element = EcoreUtil.resolve(element, getEditingDomain().getResourceSet());
						}
						IDE.openEditor(RepositoryView.this.getSite().getPage(), WorkspaceSynchronizer.getFile(element.eResource()));
						refreshView();
					} catch (PartInitException e) {
						e.printStackTrace();
					}
				}
				
			}
		});
		plansViewer.getControl().addFocusListener(new FocusAdapter(){
			@Override
			public void focusGained(FocusEvent e) {
				updateGlobalActions(plansViewer);
			}
			@Override
			public void focusLost(FocusEvent e) {
				updateGlobalActions(null);
			}
		});
		
		viewers = new StructuredViewer[5];
		viewers[0] = plansViewer;
		
		// Add the TabItem for the plantypes
		tabItem = new TabItem(folder, SWT.NONE);
		tabItem.setText("Plantypes");
		tabItem.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_PLANTYPE_16));
		
		// Create the viewer for the plantypes		
		planTypesViewer = new TableViewer(folder, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL);
		planTypesViewer.setContentProvider(new ViewContentProvider("pty"));
		planTypesViewer.setLabelProvider(new ViewLabelProvider());
		planTypesViewer.setSorter(new ViewerSorter());
		planTypesViewer.addDoubleClickListener(new IDoubleClickListener(){
			public void doubleClick(DoubleClickEvent event) {
				Object selection = ((IStructuredSelection)event.getSelection()).getFirstElement();
				if(selection instanceof EObject){
					PlanType type = (PlanType)selection;
					
					// Create a plantype configuration wizard and initialize it with the plantype
					PMLPlanTypeConfigurationWizard wiz = new PMLPlanTypeConfigurationWizard(type);
					
					WizardDialog dialog = new WizardDialog(planTypesViewer.getControl().getShell(), wiz);
					
					dialog.setBlockOnOpen(true);
					dialog.open();
				}
				
			}
		});
		planTypesViewer.getControl().addFocusListener(new FocusAdapter(){
			@Override
			public void focusGained(FocusEvent e) {
				updateGlobalActions(planTypesViewer);
			}
			@Override
			public void focusLost(FocusEvent e) {
				updateGlobalActions(null);
			}
		});
		tabItem.setControl(planTypesViewer.getControl());
		
		viewers[1] = planTypesViewer;
		
		// Add the TabItem for the behaviours
		tabItem = new TabItem(folder, SWT.NONE);
		tabItem.setText("Behaviours");
		tabItem.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_BEHAVIOUR_16));
		
		// Create the viewer for the behaviours
		behaviourViewer = new TreeViewer(folder, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL);
		behaviourViewer.setContentProvider(new BehaviourTreeContentProvider());
		behaviourViewer.setLabelProvider(new BehaviourTreeLabelProvider());
		behaviourViewer.setSorter(new ViewerSorter());
		behaviourViewer.getControl().addFocusListener(new FocusAdapter(){
			@Override
			public void focusGained(FocusEvent e) {
				//War auskommentiert um das löschen zu verhindern! Verbietet dann alle actions
				updateGlobalActions(behaviourViewer);
			}
			@Override
			public void focusLost(FocusEvent e) {
				//War auskommentiert um das löschen zu verhindern!
				updateGlobalActions(null);
			}
		});
		tabItem.setControl(behaviourViewer.getControl());
		
		viewers[2] = behaviourViewer;
		
		// Add the TabItem for the taskrepository
		tabItem = new TabItem(folder, SWT.NONE);
		tabItem.setText("Tasks");
		tabItem.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_TASK_16));
		
		// Create the viewer for the behaviours
		taskrepositoryViewer = new TableViewer(folder, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL);
		taskrepositoryViewer.setContentProvider(new TaskrepositoryContentProvider());
		taskrepositoryViewer.setLabelProvider(new TaskrepositoryLabelProvider());
		taskrepositoryViewer.setSorter(new ViewerSorter());
		taskrepositoryViewer.getControl().addFocusListener(new FocusAdapter(){
			@Override
			public void focusGained(FocusEvent e) {
				updateGlobalActions(taskrepositoryViewer);
			}
			@Override
			public void focusLost(FocusEvent e) {
				updateGlobalActions(null);
			}
		});
		tabItem.setControl(taskrepositoryViewer.getControl());
		
		viewers[3] = taskrepositoryViewer;
		
		// Add the TabItem for the planningProblems
		tabItem = new TabItem(folder, SWT.NONE);
		tabItem.setText("PlanningProblem");
		tabItem.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_PLANNING_PROBLEM_16));
		
		// Create the viewer for the plans
		planningProblemsViewer = new TableViewer(folder, SWT.BORDER | SWT.MULTI | SWT.H_SCROLL | SWT.V_SCROLL);
		planningProblemsViewer.setContentProvider(new ViewContentProvider("pp"));
		planningProblemsViewer.setLabelProvider(new ViewLabelProvider());
		planningProblemsViewer.setSorter(new ViewerSorter());
		tabItem.setControl(planningProblemsViewer.getControl());
		planningProblemsViewer.addDoubleClickListener(new IDoubleClickListener(){
			public void doubleClick(DoubleClickEvent event) {
				Object selection = ((IStructuredSelection)event.getSelection()).getFirstElement();
				if(selection instanceof EObject){
					PlanningProblem pp = (PlanningProblem)selection;
					
					// Create a plantype configuration wizard and initialize it with the plantype
					PMLPlanningProblemConfigurationWizard wiz = new PMLPlanningProblemConfigurationWizard(pp);
					
					WizardDialog dialog = new WizardDialog(planningProblemsViewer.getControl().getShell(), wiz);
					
					dialog.setBlockOnOpen(true);
					dialog.open();
				}
				
			}
		});
		planningProblemsViewer.getControl().addFocusListener(new FocusAdapter(){
			@Override
			public void focusGained(FocusEvent e) {
				updateGlobalActions(planningProblemsViewer);
			}
			@Override
			public void focusLost(FocusEvent e) {
				updateGlobalActions(null);
			}
		});
		
		viewers[4] = planningProblemsViewer;
		
		makeActions();

		// Create a SelectionProviderMediator 
		SelectionProviderMediator med = new SelectionProviderMediator(viewers,null);
		getSite().setSelectionProvider(med);
		hookContextMenu();
		
		// Initialize the input
		createInitialInput();
	}
	
	private void updateGlobalActions(StructuredViewer viewer) {
		IActionBars bars = getViewSite().getActionBars();
		if(behaviourViewer.equals(viewer)){
			bars.setGlobalActionHandler(ActionFactory.DELETE.getId(), deleteBehaviourConfigAction);
		}else if(taskrepositoryViewer.equals(viewer)){
			bars.setGlobalActionHandler(ActionFactory.DELETE.getId(), deleteTaskAction);
		}else if(plansViewer.equals(viewer)){
			bars.setGlobalActionHandler(ActionFactory.DELETE.getId(), deletePlan);
		}else if(planTypesViewer.equals(viewer)){
			bars.setGlobalActionHandler(ActionFactory.DELETE.getId(), deletePlantypes);
		}
		
		bars.updateActionBars();
	}

	private DeleteBehaviourConfigurationAction deleteBehaviourConfigAction;
	private DeleteTaskFromRepositoryAction deleteTaskAction;
	private DeletePlanFromRepositoryAction deletePlan;
	private DeletePlanTypesFromRepositryAction deletePlantypes;
	
	private void makeActions() {
		deleteBehaviourConfigAction = new DeleteBehaviourConfigurationAction(behaviourViewer);
		deleteTaskAction = new DeleteTaskFromRepositoryAction(taskrepositoryViewer);
		deletePlan = new DeletePlanFromRepositoryAction(plansViewer);
		deletePlantypes = new DeletePlanTypesFromRepositryAction(planTypesViewer);
	}

	private void createInitialInput(){
		// Register a listener to the workspace
		IWorkspace workspace = ResourcesPlugin.getWorkspace();
		workspace.addResourceChangeListener(getWorkspaceChangeListener());
		
		// Set the input for all viewers
		for(int i=0; i < viewers.length; i++){
			StructuredViewer viewer = viewers[i];
			// Add the drag support
			addDragSupport(viewer);
			
			viewers[i].setInput(workspace.getRoot());
		}
	}

	private void hookContextMenu() {
		MenuManager menuMgr = new MenuManager();
		
		Menu contextMenu = menuMgr.createContextMenu(getSite().getShell());
		behaviourViewer.getControl().setMenu(contextMenu);
		plansViewer.getControl().setMenu(contextMenu);
		planTypesViewer.getControl().setMenu(contextMenu);
		taskrepositoryViewer.getControl().setMenu(contextMenu);
		planningProblemsViewer.getControl().setMenu(contextMenu);
		
		// Add the marker where other plugins can contribute new actions
		menuMgr.add(new GroupMarker(IWorkbenchActionConstants.MB_ADDITIONS));
		
		getSite().registerContextMenu(PlanDesignerConstants.PML_REPOSITORY_ID +".repositoryContextMenu", menuMgr, getSite().getSelectionProvider());
	}

	/**
	 * Passing the focus request to the viewer's control.
	 */
	public void setFocus() {
		folder.setFocus();
	}

	private IResourceChangeListener getWorkspaceChangeListener() {
		if (this.workspaceChangeListener == null) {
			this.workspaceChangeListener = new IResourceChangeListener() {
				public void resourceChanged(IResourceChangeEvent event) {
					refreshView();
				}
			};
		}
		return workspaceChangeListener;
	}
	
	@Override
	public void dispose() {
		IWorkspace workspace = ResourcesPlugin.getWorkspace();
		workspace.removeResourceChangeListener(getWorkspaceChangeListener());
		adapterFactory.dispose();
		
		// TODO: Unload the loaded resources here?
		
//		try {
//			getEditingDomain().getCommandStack().execute(new RecordingCommand(getEditingDomain()){
//				@Override
//				protected void doExecute() {
//					for(Resource r : loadedResources){
//						r.getContents().clear();
//						r.unload();
//						editingDomain.getResourceSet().getResources().remove(r);
//					}
//					loadedResources.clear();
//				}
//			},Collections.singletonMap(
//			        Transaction.OPTION_UNPROTECTED, Boolean.TRUE));
//		} catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		} catch (RollbackException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
		
		
		super.dispose();
	}
	
	@SuppressWarnings("rawtypes")
	@Override
	public Object getAdapter(Class adapter) {
		if (adapter == IPropertySheetPage.class)
			return createPropertySheetPage();
		return super.getAdapter(adapter);
	}
	
	/**
	 * Returns the undoable <code>PropertySheetPage</code> for this editor.
	 * 
	 * @return the undoable <code>PropertySheetPage</code>
	 */
	protected Page createPropertySheetPage() {
		return new PMLTabbedPropertySheetPage(this);
	}

	public String getContributorId() {
		return PlanDesignerConstants.PLAN_EDITOR_ID;
	}

	public EMF2GEFCommandStack getEMFCommandStack() {
		if(this.commandStack == null){
			this.commandStack = (EMF2GEFCommandStack)getEditingDomain().getCommandStack();
		}
		
		return this.commandStack;
	}

	protected MultiObjectNotificationAdapter getBehaviourNotificationAdapter() {
		if(behaviourNotificationAdapter == null){
			behaviourNotificationAdapter = new MultiObjectNotificationAdapter(){
				@Override
				public void doNotify(Notification n) {
					// Since we only display PlanElement names in our view it is sufficient
					// to react to this feature
					if(n.getFeature().equals(AlicaPackage.eINSTANCE.getPlanElement_Name()))
						refreshView();
				}
				
			};
		}
		return behaviourNotificationAdapter;
	}
	
	private void refreshView(){
		Display.getDefault().asyncExec(new Runnable(){
			public void run() {
				for (Viewer v : viewers)
					v.refresh();
			}
		});
	}
	
	protected MultiObjectNotificationAdapter getTaskNotificationAdapter() {
		if(taskNotificationAdapter == null){
			taskNotificationAdapter = new MultiObjectNotificationAdapter(){
				@Override
				public void doNotify(Notification n) {
					// Since we only display PlanElement names in our view it is sufficcient
					// to react to this feature
					if(n.getFeature().equals(AlicaPackage.eINSTANCE.getPlanElement_Name()))
						refreshView();
				}
				
			};
		}
		return taskNotificationAdapter;
	}

	protected MultiObjectNotificationAdapter getPlanNotificationAdapter() {
		if(planNotificationAdapter == null){
			planNotificationAdapter = new MultiObjectNotificationAdapter(){
				@Override
				public void doNotify(Notification n) {
					// Since we only display PlanElement names in our view it is sufficcient
					// to react to this feature
					if(n.getFeature().equals(AlicaPackage.eINSTANCE.getPlanElement_Name()))
						refreshView();
				}
				
			};
		}
		return planNotificationAdapter;
	}
	
	protected MultiObjectNotificationAdapter getPlanningNotificationAdapter() {
		if(planningProblemNotificationAdapter == null){
			planningProblemNotificationAdapter = new MultiObjectNotificationAdapter(){
				@Override
				public void doNotify(Notification n) {
					// Since we only display PlanElement names in our view it is sufficient
					// to react to this feature
					if(n.getFeature().equals(AlicaPackage.eINSTANCE.getPlanningProblem().getName()))
						refreshView();
				}
				
			};
		}
		return planningProblemNotificationAdapter;
	}

	protected MultiObjectNotificationAdapter getPlanTypeNotificationAdapter() {
		if(planTypeNotificationAdapter == null){
			planTypeNotificationAdapter = new MultiObjectNotificationAdapter(){
				@Override
				public void doNotify(Notification n) {
					refreshView();
				}
				
			};
		}
		return planTypeNotificationAdapter;
	}

	public PMLTransactionalEditingDomain getEditingDomain() {
		if(editingDomain == null){
			// Establish a connecton to the editingDomain
			editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		}
		return editingDomain;
	}

	public AdapterFactory getAdapterFactory() {
		return adapterFactory;
	}
}