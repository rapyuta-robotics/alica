package de.uni_kassel.vs.cn.plandesigner.condition.defaultPlugin;


import java.util.LinkedList;
import java.util.List;

import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.viewers.ViewerSorter;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.layout.FormLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Variable;



public class PMLModifyConditionVariables extends WizardPage {
	private class VarListLabelProvider extends LabelProvider{
	/*	@Override
		public Image getImage(Object element) {
			if(element instanceof Plan){
				return PlanDesignerActivator.getDefault().getImageRegistry().get(IEditorConstants.ICON_PLAN_16);
			}else
				return null;
		}
		*/
		@Override
		public String getText(Object element) {
			if(element instanceof Variable){
				return ((Variable)element).getName();
			}else
				return "Unrecognized element";
		}
	}
	
	private class VarListContentProvider implements IStructuredContentProvider{
		
		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
		}

		public Object[] getElements(Object inputElement) {
			return ((List<Variable>)inputElement).toArray();
		}
	}
	
	private TableViewer varsViewer;
	
	private TableViewer conditionTableViewer;
	
	private List<Variable> varsViewerList;
	
	private List<Variable> conditionViewerList;
		
	
	private Condition type;
	private AbstractPlan plan;
		
	

	public PMLModifyConditionVariables(Condition type, AbstractPlan plan) {
		super("Variables");
		setTitle("Choose Variables");		
		this.type = type;
		this.plan = plan;
	}

	public void createControl(Composite parent) {
		Composite container = new Composite(parent, SWT.NONE);
		container.setLayout(new GridLayout(3,false));
		
		// Create elements for the left (plan) side
		Label availablePlansLabel = new Label(container, SWT.NONE);
		availablePlansLabel.setText("Available Variables");
		
		Label conditionLabel = new Label(container, SWT.NONE);
		conditionLabel.setText("Used By Condition");
		
		varsViewer = new TableViewer(container);
		varsViewer.setSorter(new ViewerSorter());
		varsViewer.setContentProvider(new VarListContentProvider());
		varsViewer.setLabelProvider(new VarListLabelProvider());
		/*varsViewer.addDoubleClickListener(new IDoubleClickListener(){
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
		});*/
		
		
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
				move(varsViewer.getSelection(), varsViewerList, conditionViewerList);
				dialogChanged();
			}
		});
		
		Button addAllButton = new Button(buttonsContainer, SWT.PUSH);
		addAllButton.setText("Add all >>");
		addAllButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				moveAll(varsViewerList, conditionViewerList);
				dialogChanged();
			}
		});
		Button upButton = new Button(buttonsContainer, SWT.PUSH);
		upButton.setText("Up");
		upButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				moveup(conditionTableViewer.getSelection(),conditionViewerList);
				dialogChanged();
			}
		});
		Button downButton = new Button(buttonsContainer, SWT.PUSH);
		downButton.setText("Down");
		downButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				movedown(conditionTableViewer.getSelection(),conditionViewerList);
				dialogChanged();
			}
		});
		Button removeButton = new Button(buttonsContainer, SWT.PUSH);
		removeButton.setText("< Remove");
		removeButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				move(conditionTableViewer.getSelection(), conditionViewerList, varsViewerList);
				dialogChanged();
			}
		});
		
		Button removeAllButton = new Button(buttonsContainer, SWT.PUSH);
		removeAllButton.setText("<< Remove all");
		removeAllButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				moveAll(conditionViewerList, varsViewerList);
				dialogChanged();
			}
		});
		
		
		
		
		// Create the elements for the right (new plantype) side
		conditionTableViewer = new TableViewer(container);
		//conditionTableViewer.setSorter(new ViewerSorter());
		conditionTableViewer.setContentProvider(new VarListContentProvider());
		conditionTableViewer.setLabelProvider(new VarListLabelProvider());
		
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
		fData.top = new FormAttachment(addAllButton,30);
		fData.width = 100;
		upButton.setLayoutData(fData);

		fData = new FormData();
		fData.top = new FormAttachment(upButton);
		fData.width = 100;
		downButton.setLayoutData(fData);

		
		fData = new FormData();
		fData.top = new FormAttachment(downButton, 30);
		fData.width = 100;
		removeButton.setLayoutData(fData);
				
		
		fData = new FormData();
		fData.top = new FormAttachment(removeButton);
		fData.width = 100;
		removeAllButton.setLayoutData(fData);
		
		// Do the layout for the whole container
		availablePlansLabel.setLayoutData(new GridData(SWT.BEGINNING,SWT.CENTER,true,false,2,1));
		conditionLabel.setLayoutData(new GridData(SWT.BEGINNING, SWT.CENTER,true,false));
		
		GridData gData = new GridData(SWT.FILL,SWT.FILL,true,true);
		gData.widthHint = 170;
		gData.heightHint = 250;
		varsViewer.getTable().setLayoutData(gData);
		
		buttonsContainer.setLayoutData(new GridData(SWT.FILL,SWT.CENTER,false,true));
		
		gData = new GridData(SWT.FILL,SWT.FILL,true,true);
		gData.widthHint = 170;
		gData.heightHint = 250;
		conditionTableViewer.getTable().setLayoutData(gData);
		
		setControl(container);
		initializeInput();
		dialogChanged();
	}
	
	
	private void moveAll(List<Variable> from, List<Variable> to){
		to.addAll(from);
		from.clear();
		
		varsViewer.refresh();
		conditionTableViewer.refresh();
	}
	
	
	private void move(ISelection vars, List<Variable> from, List<Variable> to){
		IStructuredSelection sel = (IStructuredSelection)vars;
		List<Variable> selectedVars = sel.toList();
		from.removeAll(selectedVars);
		to.addAll(selectedVars);
		
		varsViewer.refresh();
		conditionTableViewer.refresh();
	}
	private void moveup(ISelection vars, List<Variable> from) {
		IStructuredSelection sel = (IStructuredSelection)vars;
		List<Variable> selectedVars = sel.toList();
		if (selectedVars.size()>0) {
			int idx = from.indexOf(selectedVars.get(0));
			from.removeAll(selectedVars);
			idx = Math.max(0,idx-1);
			from.addAll(idx, selectedVars);
			conditionTableViewer.refresh();
		}
	}
	private void movedown(ISelection vars, List<Variable> from) {
		IStructuredSelection sel = (IStructuredSelection)vars;
		List<Variable> selectedVars = sel.toList();
		if (selectedVars.size()>0) {
			int idx = from.indexOf(selectedVars.get(0));
			from.removeAll(selectedVars);
			idx = Math.min(from.size(),idx+1);
			from.addAll(idx, selectedVars);
			conditionTableViewer.refresh();
		}		
	}
	
	private void initializeInput(){
		
		varsViewerList = new LinkedList<Variable>();
		conditionViewerList = new LinkedList<Variable>();
		
		if(type != null){
			List<Variable> currentVars = type.getVars();
			conditionViewerList.addAll(currentVars);
			
			List<Variable> possibleVars = plan.getVars();
			varsViewerList.addAll(possibleVars);
			varsViewerList.removeAll(currentVars);
		}
		conditionTableViewer.setInput(conditionViewerList);
		
		varsViewer.setInput(varsViewerList);
	}
	
	/**
	 * Ensures that both text fields are set.
	 */
	private void dialogChanged() {
		
		updateStatus(null);
	}

	private void updateStatus(String message) {
		setErrorMessage(message);
		setPageComplete(message == null);
	}

	public List<Variable> getConditionViewerList() {
		return conditionViewerList;
	}

}
