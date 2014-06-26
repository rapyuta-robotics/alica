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
package de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.workspace.util.WorkspaceSynchronizer;
import org.eclipse.jface.viewers.CellEditor;
import org.eclipse.jface.viewers.CheckboxCellEditor;
import org.eclipse.jface.viewers.DoubleClickEvent;
import org.eclipse.jface.viewers.ICellModifier;
import org.eclipse.jface.viewers.IDoubleClickListener;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ITableLabelProvider;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TextCellEditor;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.viewers.ViewerSorter;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.events.MouseListener;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.layout.FormLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.TableItem;
import org.eclipse.ui.IEditorInput;
import org.eclipse.ui.IWorkbenchPage;
import org.eclipse.ui.IWorkbenchWindow;
import org.eclipse.ui.PartInitException;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.ide.IDE;
import org.eclipse.ui.part.FileEditorInput;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;

public class PMLComposePlanTypeWizardPage extends WizardPage {
	
	private class PlanListLabelProvider extends LabelProvider{
		@Override
		public Image getImage(Object element) {
			if(element instanceof Plan){
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_PLAN_16);
			}else
				return null;
		}
		
		@Override
		public String getText(Object element) {
			if(element instanceof Plan){
				return ((Plan)element).getName();
			}else
				return "Unrecognized element";
		}
	}
	private class AnnotatedPlanListLabelProvider extends LabelProvider implements ITableLabelProvider{
		@Override
		public Image getImage(Object element) {
			if(element instanceof Plan){
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_PLAN_16);
			}else
				return null;
		}
				
		public String getColumnText(Object element, int col) {
			if (col == 1) {
				if(element instanceof AnnotatedPlan){
					return ((AnnotatedPlan)element).getPlan().getName();
				}else
					return "Unrecognized element";
			} else { return "";}
		}
		public Image getColumnImage(Object element, int columnIndex)
        {
        	if (columnIndex == 0)
        	{
        		AnnotatedPlan temp = (AnnotatedPlan) element;
        		if (temp.isActivated())
        			return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_SUCCESS_POINT_15);
        			
        		else
        			return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_FAILURE_TRANSITION_16);
        	}
        	else return null;
        }
	}
	private class AnnotatedPlanListContentProvider implements IStructuredContentProvider{
		
		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
		}

		public Object[] getElements(Object inputElement) {
			return ((Set<AnnotatedPlan>)inputElement).toArray();
		}
	}
	private class PlanListContentProvider implements IStructuredContentProvider{
		
		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
		}

		public Object[] getElements(Object inputElement) {
			return ((Set<Plan>)inputElement).toArray();
		}
	}
	
	private TableViewer plansViewer;
	
	private TableViewer plantypeTableViewer;
	
	private Set<Plan> plansViewerList;
	
	private Set<AnnotatedPlan> plantypeViewerList;
	
	private PlanType type;
	
	private PMLTransactionalEditingDomain domain;
	
	public PMLComposePlanTypeWizardPage(PMLTransactionalEditingDomain domain, ResourceSet rSet) {
		this(domain, rSet, null);
		
	}

	public PMLComposePlanTypeWizardPage(PMLTransactionalEditingDomain domain, ResourceSet rSet, PlanType type) {
		super("New Plantype");
		setTitle("Configure the plantype");
		this.type = type;
		this.domain = domain;
	}

	public void createControl(Composite parent) {
		Composite container = new Composite(parent, SWT.NONE);
		container.setLayout(new GridLayout(3,false));
		
		// Create elements for the left (plan) side
		Label availablePlansLabel = new Label(container, SWT.NONE);
		availablePlansLabel.setText("Available Plans");
		
		Label planTypeLabel = new Label(container, SWT.NONE);
		planTypeLabel.setText("Plantype");
		
		plansViewer = new TableViewer(container);
		plansViewer.setSorter(new ViewerSorter());
		plansViewer.setContentProvider(new PlanListContentProvider());
		plansViewer.setLabelProvider(new PlanListLabelProvider());
		plansViewer.addDoubleClickListener(new IDoubleClickListener(){
			public void doubleClick(DoubleClickEvent event) {
				Object selection = ((IStructuredSelection)event.getSelection()).getFirstElement();
				if(selection instanceof EObject){
					try {
						EObject element = (EObject)selection;
						IWorkbenchPage page =
							PlatformUI.getWorkbench().getActiveWorkbenchWindow().getActivePage();
						IDE.openEditor(page, WorkspaceSynchronizer.getFile(element.eResource()));
					} catch (PartInitException e) {
						PlanDesignerActivator.getDefault().getLog().log(
								new Status(IStatus.ERROR, PlanDesignerActivator.PLUGIN_ID,"Error while opening plan",e));
					}
				}
				
			}
		});
		
		
		// Create the buttons for the mid
		Composite buttonsContainer = new Composite(container, SWT.NONE);
		FormLayout fLayout = new FormLayout();
		fLayout.spacing = 15;
		buttonsContainer.setLayout(fLayout);
		
		Button addButton = new Button(buttonsContainer, SWT.PUSH);
		addButton.setText("Add >");
		addButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				// Move the selected plan from the plansViewerList to
				// the planTypeViewerList
				moveTo(plansViewer.getSelection(), plansViewerList, plantypeViewerList);
				dialogChanged();
			}
		});
		
		Button addAllButton = new Button(buttonsContainer, SWT.PUSH);
		addAllButton.setText("Add all >>");
		addAllButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				moveAllTo(plansViewerList, plantypeViewerList);
				dialogChanged();
			}
		});
		
		Button removeButton = new Button(buttonsContainer, SWT.PUSH);
		removeButton.setText("< Remove");
		removeButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				moveFrom(plantypeTableViewer.getSelection(), plantypeViewerList, plansViewerList);
				dialogChanged();
			}
		});
		
		Button removeAllButton = new Button(buttonsContainer, SWT.PUSH);
		removeAllButton.setText("<< Remove all");
		removeAllButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				moveAllFrom(plantypeViewerList, plansViewerList);
				dialogChanged();
			}
		});
		
		// Create the elements for the right (new plantype) side
		plantypeTableViewer = new TableViewer(container);
		
		plantypeTableViewer.setSorter(new ViewerSorter());
		
		
		Table table = plantypeTableViewer.getTable();
    	table.setLinesVisible(true);
    	table.setHeaderVisible(true);
    	table.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
    	
    	TableColumn colSelection = new TableColumn(table, SWT.NONE);
    	colSelection.setWidth(60);
    	colSelection.setText("Active");

    	TableColumn colPlanName = new TableColumn(table, SWT.NONE);
    	colPlanName.setWidth(140);
    	colPlanName.setText("Plan");
    	
		plantypeTableViewer.setContentProvider(new AnnotatedPlanListContentProvider());
		plantypeTableViewer.setLabelProvider(new AnnotatedPlanListLabelProvider());
		
		plantypeTableViewer.setCellEditors
	    	(
				new CellEditor[]
				{
					new CheckboxCellEditor(table), new AnPlanCellEditor()
				}
	    	);   
	        plantypeTableViewer.setCellModifier(new ICellModifier()
	        {
	    		public boolean canModify(Object element, String property) 
	    		{
	    			return true;
	    		}
	    		
	    		public Object getValue(Object element, String property) 
	    		{               
	    			if (property.equals("colSelection")) {
	                	return ((AnnotatedPlan)element).isActivated();
	    			}
	                else{
	                	//Open the AnnotatedPlan by Click 
	                	if(element instanceof AnnotatedPlan){
	                		URI eUri = ((AnnotatedPlan) element).getPlan().eResource().getURI();
	                		if (eUri.isPlatformResource()) {
	                			IEditorInput editorInput = new FileEditorInput((IFile) ResourcesPlugin.getWorkspace().getRoot().findMember(eUri.toPlatformString(true)));
	                			IWorkbenchWindow window = PlatformUI.getWorkbench().getActiveWorkbenchWindow();
	                			IWorkbenchPage page = window.getActivePage();
	                			try {
									page.openEditor(editorInput, "de.uni_kassel.vs.cn.planDesigner.ui.pmlEditor");
								} catch (PartInitException e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}   
	                		}
	                	}
	                	return null;
	                }
	    		}

	    		public void modify(Object element, String property, Object value) 
	    		{
	    			//System.out.println("element to modify is:" + element.toString());
	    			//TableItem t = (TableItem)element;
	    			//System.out.println("element to modify is:" + t.getData().toString());
	    			
	    			// Save the resource where the plantype we are configuring is in
	    			final AnnotatedPlan p = (AnnotatedPlan)((TableItem)element).getData();
	    			final Boolean val = (Boolean)value;
	    			
	    			domain.getCommandStack().execute(new RecordingCommand(domain){
						@Override
						protected void doExecute() {
								p.setActivated(val);
						}
					});
	    			
	    			dialogChanged();
	    			plantypeTableViewer.refresh();

	    		}
	    	});

	    plantypeTableViewer.setColumnProperties(new String[]{"colSelection", ""});
	    

		// Do the layout for the buttonsContainer
		FormData fData = new FormData();
		fData.top = new FormAttachment();
		fData.width = 100;
		
		addButton.setLayoutData(fData);
		
		fData = new FormData();
		fData.top = new FormAttachment(addButton);
		fData.width = 100;
		addAllButton.setLayoutData(fData);
		
		fData = new FormData();
		fData.top = new FormAttachment(addAllButton, 50);
		fData.width = 100;
		removeButton.setLayoutData(fData);
		
		fData = new FormData();
		fData.top = new FormAttachment(removeButton);
		fData.width = 100;
		removeAllButton.setLayoutData(fData);
		
		// Do the layout for the whole container
		availablePlansLabel.setLayoutData(new GridData(SWT.BEGINNING,SWT.CENTER,true,false,2,1));
		planTypeLabel.setLayoutData(new GridData(SWT.BEGINNING, SWT.CENTER,true,false));
		
		GridData gData = new GridData(SWT.FILL,SWT.FILL,true,true);
		gData.widthHint = 170;
		gData.heightHint = 250;
		plansViewer.getTable().setLayoutData(gData);
		
		buttonsContainer.setLayoutData(new GridData(SWT.FILL,SWT.CENTER,false,true));
		
		gData = new GridData(SWT.FILL,SWT.FILL,true,true);
		gData.widthHint = 170;
		gData.heightHint = 250;
		plantypeTableViewer.getTable().setLayoutData(gData);
		
		setControl(container);
		initializeInput();
		dialogChanged();
	}
	
	/**
	 * Moves all plans from one set to the other and calls refresh
	 * on both viewers.
	 * @param from
	 * @param to
	 */
	private void moveAllTo(Set<Plan> from, Set<AnnotatedPlan> to){
		for(Plan p : from) {
			AnnotatedPlan ap = AlicaFactory.eINSTANCE.createAnnotatedPlan();
			ap.setActivated(true);
			ap.setPlan(p);
			to.add(ap);
		}
			//to.addAll(from);		
		from.clear();
		plansViewer.refresh();
		plantypeTableViewer.refresh();
	}
	private void moveAllFrom(Set<AnnotatedPlan> from, Set<Plan> to) {
		for(AnnotatedPlan ap : from) {			
			to.add(ap.getPlan());
		}
		from.clear();
		plansViewer.refresh();
		plantypeTableViewer.refresh();
	}
	
	/**
	 * Removes the given plans from the <code>from</code> set and 
	 * adds it to the <code>to</code> set. After that it calls
	 * refresh on both viewers. 
	 * @param plan
	 * @param from
	 * @param to
	 */
	private void moveTo(ISelection plans, Set<Plan> from, Set<AnnotatedPlan> to){
		IStructuredSelection sel = (IStructuredSelection)plans;
		List<Plan> selectedPland = sel.toList();
		from.removeAll(selectedPland);
		for(Plan p : selectedPland) {
			AnnotatedPlan ap = AlicaFactory.eINSTANCE.createAnnotatedPlan();
			ap.setActivated(true);
			ap.setPlan(p);
			to.add(ap);
		}
		//to.addAll(selectedPland);
		
		plansViewer.refresh();
		plantypeTableViewer.refresh();
	}
	private void moveFrom(ISelection plans, Set<AnnotatedPlan> from, Set<Plan> to){
		IStructuredSelection sel = (IStructuredSelection)plans;
		List<AnnotatedPlan> selectedPland = sel.toList();
		from.removeAll(selectedPland);
		for(AnnotatedPlan p : selectedPland) {
			to.add(p.getPlan());
		}
		//to.addAll(selectedPland);
		
		plansViewer.refresh();
		plantypeTableViewer.refresh();
	}
	
	private void initializeInput(){
		// Get all planFiles within the workspace
		Set<IFile> planFiles = PlanEditorUtils.collectAllFilesWithExtension("pml");
		
		plansViewerList = new HashSet<Plan>();
		
		// Load all plans into the resourceset
		for(IFile file : planFiles)
			plansViewerList.add((Plan)domain.load(file).getContents().get(0));
		
		plantypeViewerList = new HashSet<AnnotatedPlan>();
		
		// If the the page was initialized with a plantype 
		// we will add all plans in the plantype to the 
		// plantypeViewerList and remove those from the
		// plansViewerList
		if(type != null){
			List<AnnotatedPlan> plans = type.getPlans();
			plantypeViewerList.addAll(plans);
			
			// TODO: This is not working cause the plan object from the plantype
			// are different from those which are in the plansViewerList. This is
			// caused by loading them through different editingdomains
			plansViewerList.removeAll(plans);
		}
		plantypeTableViewer.setInput(plantypeViewerList);
		
		// Set the files as input for the viewer
		plansViewer.setInput(plansViewerList);
	}
	
	/**
	 * Ensures that both text fields are set.
	 */
	private void dialogChanged() {
		if(!(plantypeTableViewer.getTable().getItemCount() > 0)){
			updateStatus("Select at least one plan to add to the plantype!");
			return;
		}
			
			
		updateStatus(null);
	}

	private void updateStatus(String message) {
		setErrorMessage(message);
		setPageComplete(message == null);
	}

	public Set<AnnotatedPlan> getPlantypeViewerList() {
		return plantypeViewerList;
	}
	
	
	//Do not delete
	public class AnPlanCellEditor extends CellEditor{

		@Override
		protected Control createControl(Composite parent) {
			// TODO Auto-generated method stub
			return null;
		}

		@Override
		protected Object doGetValue() {
			return null;
		}

		@Override
		protected void doSetFocus() {
			// TODO Auto-generated method stub
			
		}

		@Override
		protected void doSetValue(Object value) {
			// TODO Auto-generated method stub
			
		}
		
	}
	
	
}
