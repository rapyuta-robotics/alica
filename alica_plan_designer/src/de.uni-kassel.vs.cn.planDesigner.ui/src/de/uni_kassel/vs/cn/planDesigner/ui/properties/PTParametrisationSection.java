package de.uni_kassel.vs.cn.planDesigner.ui.properties;


import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.util.BasicEList;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.RunnableWithResult;
import org.eclipse.jface.viewers.CellEditor;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.ColumnViewer;
import org.eclipse.jface.viewers.ComboBoxCellEditor;
import org.eclipse.jface.viewers.EditingSupport;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.viewers.ViewerSorter;
import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.CLabel;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AnnotatedPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Parametrisation;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;
import de.uni_kassel.vs.cn.planDesigner.alica.Variable;


public class PTParametrisationSection extends PMLPropertySection {
	private class TableContentProvider implements IStructuredContentProvider{

		public Object[] getElements(final Object inputElement) {
			Object[] result = null	;			
			try {
				result = (Object[])getEditingDomain().runExclusive(new RunnableWithResult.Impl<Object[]>(){
					public void run() {
						PlanType s = (PlanType)inputElement;
						EList<Parametrisation> params = new BasicEList<Parametrisation>();
						params.addAll(s.getParametrisation());
						for(AnnotatedPlan subap : s.getPlans()) {
							AbstractPlan subp = subap.getPlan();
							for(Variable subv : subp.getVars()) {
								boolean found = false;
								for(Parametrisation pp : params) {
									if (pp.getSubplan().getId()==subp.getId() && pp.getSubvar().getId()==subv.getId()) {
										found = true;
										break;
									}
								}
								if (!found) {
									Parametrisation newp = AlicaFactory.eINSTANCE.createParametrisation();
									newp.setSubvar(subv);
									newp.setSubplan(subp);
									params.add(newp);
								}
							}
						}
						setResult(params.toArray());
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
	
	
	private class ParametrisationEditingSupport extends EditingSupport{
		protected EList<Variable> choices;
		
		public ParametrisationEditingSupport(ColumnViewer viewer) {			
			super(viewer);						
		}

		@Override
		protected boolean canEdit(Object element) {
			return PTParametrisationSection.this.isEditable();
		}

		@Override
		protected CellEditor getCellEditor(Object element) {
			choices = ((PlanType)getViewer().getInput()).getVars();
			String[] items = new String[choices.size()+1];
			items[0] = "none";
			for (int i=0; i < choices.size(); i++) {
				items[i+1]=choices.get(i).getName();
			}			
			return new ComboBoxCellEditor((Composite)getViewer().getControl(), items);			
		}

		@Override
		protected Object getValue(final Object element) {
			//System.out.println("getValue "+element.getClass());
			Parametrisation p = (Parametrisation)element;
			if (p.getVar()==null) return 0;			
			return choices.indexOf(p.getVar())+1;		
		}

		@Override
		protected void setValue(final Object element, final Object value) {
			//System.out.println("set value: "+element +"\t"+value);
			CompoundCommand cmp = new CompoundCommand("Modify Parametrisation");
			
			// Append a command that will update the view since the EMap doesn't fire
			// any modifications made to the model. So we wouldn't see the change
			cmp.append(new AbstractCommand(){
				// This command will actually modify the model
				RecordingCommand cmd = new RecordingCommand(getEditingDomain()){
					@Override
					protected void doExecute() {
						int index = ((Integer)value).intValue();
						Parametrisation p =(Parametrisation)element;
						EList<Parametrisation> pl = ((PlanType)getViewer().getInput()).getParametrisation();
						if (index==0) {
							p.setVar(null);
							pl.remove(p);
						} else {
							EList<Variable> cv = ((PlanType)getViewer().getInput()).getVars();
							
							if (cv.size() > index-1) {
								p.setVar(cv.get(index-1));
								if(!pl.contains(p)) pl.add(p);
								
							} else {
								throw new RuntimeException("Unexpected Modification!");
							}
						}
					}
				};
				
				public void redo() {
					cmd.redo();
					if(!PTParametrisationSection.this.getTViewer().getControl().isDisposed())
						updateView(null);
				}
				@Override
				public void undo() {
					cmd.undo();
					if(!PTParametrisationSection.this.getTViewer().getControl().isDisposed())
						updateView(null);
				}
				public void execute() {
					cmd.execute();
					if(!PTParametrisationSection.this.getTViewer().getControl().isDisposed())
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


	private TableViewer tViewer;

	
	@Override
	public void createControls(Composite parent,
			TabbedPropertySheetPage tabbedPropertySheetPage) {

		super.createControls(parent, tabbedPropertySheetPage);

		Composite composite = getWidgetFactory()
				.createFlatFormComposite(parent);
		
		CLabel l = getWidgetFactory().createCLabel(composite, "Parametrisation");


		tViewer = new TableViewer(composite, SWT.BORDER | SWT.FULL_SELECTION);
		tViewer.setColumnProperties(new String[]{"SubVariable", "Variable"});
		tViewer.getTable().setLinesVisible(true);
		tViewer.getTable().setHeaderVisible(true);


		TableViewerColumn column = new TableViewerColumn(tViewer, SWT.NONE);
		column.getColumn().setWidth(200);
		column.getColumn().setText("SubVariable");
		column.setLabelProvider(new ColumnLabelProvider() {
			public String getText(Object element) {
				Parametrisation p = (Parametrisation)element;
				return p.getSubvar().getName()+" of "+p.getSubplan().getName();				
			}
		});
		
		
		column = new TableViewerColumn(tViewer, SWT.NONE);
		column.getColumn().setWidth(200);
		column.getColumn().setText("Variable");
		column.setLabelProvider(new ColumnLabelProvider() {
			public String getText(final Object element) {
				Parametrisation p = (Parametrisation)element;
				
				if (p.getVar()==null) return "none";
				else return p.getVar().getName();
			}
		});
		column.setEditingSupport(new ParametrisationEditingSupport(tViewer));
		
		
		
		// Do some layout things
		FormData data = new FormData();
		data.left = new FormAttachment(0,0);
		l.setLayoutData(data);
		
		data = new FormData();		
		data.top = new FormAttachment(l);
		data.bottom = new FormAttachment(0,220);
//		data.right = new FormAttachment(100,0);
//		data.left = new FormAttachment(0,0);
		tViewer.getControl().setLayoutData(data);
		
		tViewer.setContentProvider(new TableContentProvider());
		tViewer.setSorter(new ViewerSorter());
		registerListeners();
	}
	
	private void registerListeners(){
		///getAddButton().addListener(SWT.Selection, getEditController());
		//getRemoveButton().addListener(SWT.Selection, getEditController());
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
		
	}
	
	public TableViewer getTViewer() {
		return tViewer;
	}
	
	@Override
	protected void updateView(Notification n) {
		getTViewer().refresh();
	}
	
	@Override
	public boolean shouldUseExtraSpace() {
		return true;
	}

	
	
	@Override
	public void refresh() {
		super.refresh();
		refreshControls();
	}
}
