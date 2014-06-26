package de.uni_kassel.vs.cn.planDesigner.ui.properties;
import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.RunnableWithResult;
import org.eclipse.jface.viewers.CellEditor;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.ColumnViewer;
import org.eclipse.jface.viewers.EditingSupport;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.jface.viewers.TextCellEditor;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.viewers.ViewerSorter;
import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.CLabel;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.Variable;

//import de.uni_kassel.vs.cn.planDesigner.ui.properties.ParametersSection.TableContentProvider;



public class VariablesSection extends PMLPropertySection {

	
	private class TableContentProvider implements IStructuredContentProvider{

		public Object[] getElements(final Object inputElement) {
			Object[] result = null	;
			try {
				result = (Object[])getEditingDomain().runExclusive(new RunnableWithResult.Impl<Object[]>(){
					public void run() {
						setResult(((AbstractPlan)inputElement).getVars().toArray());
					}
				});
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return result;
		}

		public void dispose() {
			getEditController().removeFromObjects();
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {			
		}
		
	}
	enum FIELD {
		NAME, TYPE, COMMENT
	}
	
	private class VariableEditingSupport extends EditingSupport{
		private FIELD which;
		
		public VariableEditingSupport(ColumnViewer viewer, FIELD which) {			
			super(viewer);			
			this.which = which;
		}

		@Override
		protected boolean canEdit(Object element) {
			return VariablesSection.this.isEditable();
		}

		@Override
		protected CellEditor getCellEditor(Object element) {
			return new TextCellEditor((Composite)getViewer().getControl());
		}

		@Override
		protected Object getValue(final Object element) {
			switch (this.which) {
				case NAME:
					return ((Variable)element).getName();
				case TYPE:
					return ((Variable)element).getType();
				case COMMENT:
					return ((Variable)element).getComment();
				default:
					return null;
					//throw new NotImplementedException("Field not implemented");
						
			}
		}

		@Override
		protected void setValue(final Object element, final Object value) {
			CompoundCommand cmp = new CompoundCommand("Add Variable");
			
			// Append a command that will update the view since the EMap doesn't fire
			// any modifications made to the model. So we wouldn't see the change
			cmp.append(new AbstractCommand(){
				// This command will actually modifiy the model
				RecordingCommand cmd = new RecordingCommand(getEditingDomain()){
					@Override
					protected void doExecute() {
						//EList<Variable>vars = ((AbstractPlan)getModel()).getVars();
						//System.out.println("element: "+ element + " ("+element.getClass()+")"+" value: "+value+" ("+value.getClass()+")");
						
						
						switch(which) {
							case NAME:
								((Variable)element).setName((String)value);
								break;
							case TYPE:
								((Variable)element).setType((String)value);
								break;
							case COMMENT:
								((Variable)element).setComment((String)value);
								break;
							default:
								throw new RuntimeException("Not Implemented Exception");
						}
						
					
					}
				};
				
				public void redo() {
					cmd.redo();
					if(!VariablesSection.this.getTViewer().getControl().isDisposed())
						updateView(null);
				}
				@Override
				public void undo() {
					cmd.undo();
					if(!VariablesSection.this.getTViewer().getControl().isDisposed())
						updateView(null);
				}
				public void execute() {
					cmd.execute();
					if(!VariablesSection.this.getTViewer().getControl().isDisposed())
						updateView(null);
				}
				
				@Override
				public boolean canExecute() {
					return cmd.canExecute();
				}
				
				@Override
				public boolean canUndo() {
					return cmd.canUndo();
				}
				
			});
			
			executeCommand(cmp);
		}
		
	}

	private Button addButton;

	private Button removeButton;

	private TableViewer tViewer;

	
	@Override
	public void createControls(Composite parent,
			TabbedPropertySheetPage tabbedPropertySheetPage) {

		super.createControls(parent, tabbedPropertySheetPage);

		Composite composite = getWidgetFactory()
				.createFlatFormComposite(parent);
		
		CLabel l = getWidgetFactory().createCLabel(composite, "Variables");

		addButton = getWidgetFactory().createButton(composite, "Add", SWT.PUSH);
		removeButton = getWidgetFactory().createButton(composite, "Remove",	SWT.PUSH);

		tViewer = new TableViewer(composite, SWT.BORDER | SWT.FULL_SELECTION);
		tViewer.setColumnProperties(new String[]{"Name", "Type", "Comment"});
		tViewer.getTable().setLinesVisible(true);
		tViewer.getTable().setHeaderVisible(true);


		TableViewerColumn column = new TableViewerColumn(tViewer, SWT.NONE);
		column.getColumn().setWidth(200);
		column.getColumn().setText("Name");
		column.setLabelProvider(new ColumnLabelProvider() {
			public String getText(Object element) {
				return ((Variable)element).getName();
			}
		});
		column.setEditingSupport(new VariableEditingSupport(tViewer,FIELD.NAME));
		
		column = new TableViewerColumn(tViewer, SWT.NONE);
		column.getColumn().setWidth(200);
		column.getColumn().setText("Type");
		column.setLabelProvider(new ColumnLabelProvider() {
			public String getText(final Object element) {
				return ((Variable)element).getType();				
			}
		});
		column.setEditingSupport(new VariableEditingSupport(tViewer,FIELD.TYPE));
		
		column = new TableViewerColumn(tViewer, SWT.NONE);
		column.getColumn().setWidth(200);
		column.getColumn().setText("Comment");
		column.setLabelProvider(new ColumnLabelProvider() {
			public String getText(final Object element) {
				return ((Variable)element).getComment();				
			}
		});
		column.setEditingSupport(new VariableEditingSupport(tViewer,FIELD.COMMENT));
		
		// Do some layout things
		FormData data = new FormData();
		data.left = new FormAttachment(0,0);
		l.setLayoutData(data);
		
		data = new FormData();
		data.left = new FormAttachment(0,0);
		data.top = new FormAttachment(l);
		addButton.setLayoutData(data);
		
		data = new FormData();
		data.left = new FormAttachment(addButton);
		data.top = new FormAttachment(l);
		removeButton.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(addButton,0, SWT.DEFAULT);
		data.bottom = new FormAttachment(100,0);
		data.right = new FormAttachment(100,0);
		data.left = new FormAttachment(0,0);
		tViewer.getControl().setLayoutData(data);
		
		tViewer.setContentProvider(new TableContentProvider());
		tViewer.setSorter(new ViewerSorter());
		registerListeners();
	}
	
	private void registerListeners(){
		getAddButton().addListener(SWT.Selection, getEditController());
		getRemoveButton().addListener(SWT.Selection, getEditController());
	}
	
	@Override
	protected void selectionEvent(Object source) {
		if(source.equals(getAddButton())){
			getRemoveButton().setEnabled(true);
			
			final String name = "NewVar";

			executeCommand(new RecordingCommand(getEditingDomain(), "Add Variable"){
				@Override
				protected void doExecute() {
					Variable v = AlicaFactory.eINSTANCE.createVariable();
					v.setName(name);
					((AbstractPlan)getModel()).getVars().add(v);
				}
			});
			
			getTViewer().setSelection(new StructuredSelection(name), true);
		}else if(source.equals(getRemoveButton())){
			final Object selection = ((IStructuredSelection)getTViewer().getSelection()).getFirstElement();
			int oldIdx = getTViewer().getTable().getSelectionIndex();
			if(selection != null){
				
				executeCommand(new RecordingCommand(getEditingDomain(), "Remove Parameter"){
					@Override
					protected void doExecute() {
						AbstractPlan p = (AbstractPlan)getModel();
						boolean res = p.getVars().remove(selection);						
						if (!res) throw new RuntimeException("Could not remove Variable!");
						if (p instanceof Plan) {
							((Plan)p).ensureParametrisationConsistency();							
						} else if (p instanceof PlanType) {
							((PlanType) p).ensureParametrisationConsistency();
						}
					}
				});
			}
			
			if(!hasVariables())
				getRemoveButton().setEnabled(false);
			else{
				getTViewer().getTable().setSelection(oldIdx - 1 < 0 ? oldIdx : (oldIdx-1));
			}
			
		}
	}
	
	@Override
	protected void basicSetInput(Object input) {
		super.basicSetInput(input);
		getTViewer().setInput(getModel());
		
		refreshControls();
	}
	
	/**
	 * Refreshes controls like buttons, doesn't refresh any viewers. It's intended to be used 
	 * from within refresh() to update the editable state of this section
	 */
	private void refreshControls(){
		// Enable the remove button if there are utilities
		getRemoveButton().setEnabled(false);
		getAddButton().setEnabled(false);
		
		if(isEditable()){
			getAddButton().setEnabled(true);
			if(hasVariables())
				getRemoveButton().setEnabled(true);
		}
	}
	
	public TableViewer getTViewer() {
		return tViewer;
	}
	
	private boolean hasVariables(){
		boolean result = false;
		try {
			result = (Boolean)getEditingDomain().runExclusive(new RunnableWithResult.Impl<Boolean>(){
				public void run() {
					setResult(((AbstractPlan)getModel()).getVars().size() > 0);
				}
			});
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return result;
	}
	
	@Override
	protected void updateView(Notification n) {
		getTViewer().refresh();
	}
	
	@Override
	public boolean shouldUseExtraSpace() {
		return true;
	}

	protected Button getAddButton() {
		return addButton;
	}

	protected Button getRemoveButton() {
		return removeButton;
	}
	
	@Override
	public void refresh() {
		super.refresh();
		refreshControls();
	}

}