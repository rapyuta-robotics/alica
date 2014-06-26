package de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages;


import java.util.ArrayList;

import org.eclipse.emf.common.util.EList;
import org.eclipse.jface.viewers.CellEditor;
import org.eclipse.jface.viewers.CheckboxCellEditor;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.ComboBoxCellEditor;
import org.eclipse.jface.viewers.EditingSupport;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.jface.viewers.TextCellEditor;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.layout.FormLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;

import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.IInhabitable;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.Quantifier;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;


public class PMLModifyConditionQuantifiers extends WizardPage {
	private final Condition condition;
	private final Plan plan;
	private TableViewer table;
	private Button addButton;
	
	private ArrayList<GuiQuantifier> quantifiers;
	public class GuiQuantifier {
		public IInhabitable scope;
		public ArrayList<String> sorts;
		public String type;
		public GuiQuantifier(IInhabitable i) {
			this.sorts = new ArrayList<String>();
			this.scope = i;
			this.type = "ForallAgents";
		}
	}
	public PMLModifyConditionQuantifiers(Condition condition, Plan plan) {
		super("Quantifiers");
		setTitle("Choose Quantifiers");
		this.condition = condition;
		this.plan = plan;
		this.quantifiers = new ArrayList<GuiQuantifier>();
	}
	
	
	public void createControl(Composite parent) {
	

//		parent.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		Composite createComposite = new Composite(parent, NONE); 
		createComposite.setLayout(new FormLayout());
		
		addButton = new Button(createComposite, SWT.PUSH);
		addButton.setText("Add");
		FormData data = new FormData();
		data.left = new FormAttachment(0,0);
		addButton.setLayoutData(data);
		addButton.addSelectionListener(new SelectionAdapter() {
			@Override
			public void widgetSelected(SelectionEvent e) {
				GuiQuantifier gq = new GuiQuantifier(null);
				quantifiers.add(gq);
				inputChanged();
		}});
		
		
		
		table = new TableViewer(createComposite,  SWT.BORDER | SWT.FULL_SELECTION );
		
		table.setContentProvider(new IStructuredContentProvider() {
			
			public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
				// TODO Auto-generated method stub
				
			}
			
			public void dispose() {
				// TODO Auto-generated method stub
				
			}
			
			@SuppressWarnings("unchecked")
			public Object[] getElements(Object inputElement) {
				ArrayList<GuiQuantifier> c = (ArrayList<GuiQuantifier>)inputElement;
				Object[] ret = new Object[c.size()];
				int i=0;
				for(; i<c.size();i++) {
					ret[i] = c.get(i);
				}				
				return ret;
			}
		});
		
		TableViewerColumn type = new TableViewerColumn(table,SWT.LEFT);
		type.getColumn().setWidth(120);
		//type.getColumn().setText("Name");
		
		//TableViewerColumn sorts = new TableViewerColumn(table,SWT.NONE);
		//TableViewerColumn but = new TableViewerColumn(table,SWT.NONE);
		
		type.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return "ForAllAgents";
			}
		});
		
		type.setEditingSupport(new EditingSupport(table) {
			
			@Override
			protected void setValue(Object element, Object value) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			protected Object getValue(Object element) {
				return 0;
			}
			
			@Override
			protected CellEditor getCellEditor(Object element) {
				String[] values;
				values = new String[1];
				values[0] = "ForAllAgents";
				
				return new ComboBoxCellEditor((Composite)getViewer().getControl(),values);
				//return new TextCellEditor((Composite)getViewer().getControl());
			}
			
			@Override
			protected boolean canEdit(Object element) {
				return true;
			}
		});
		TableViewerColumn in = new TableViewerColumn(table,SWT.CENTER);
		in.getColumn().setWidth(25);
		in.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return "in";
			}
		});
		TableViewerColumn scope = new TableViewerColumn(table,SWT.NONE);
		scope.getColumn().setWidth(120);
		scope.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				GuiQuantifier q = (GuiQuantifier)element;
				if (q.scope == null) return "";
				if (q.scope instanceof EntryPoint) {
					return ((EntryPoint)q.scope).getTask().getName();
				}
				return q.scope.getName();
			}
		});
		
		scope.setEditingSupport(new EditingSupport(table) {
			ArrayList<IInhabitable> elements;
			String[] values;
			
			
			
			@Override
			protected void setValue(final Object element, Object value) {
				
				final IInhabitable i = elements.get((Integer)value);
				((GuiQuantifier)element).scope = i;
				
				getViewer().update(element,null);
			}
			
			@Override
			protected Object getValue(Object element) {
				return elements.indexOf(element);			
				
			}
			
			@Override
			protected CellEditor getCellEditor(Object element) {
				elements = new ArrayList<IInhabitable>();
				Plan p = plan;
				ArrayList<String> l = new ArrayList<String>();
				elements.add(p);
				l.add("Plan: "+p.getName());
				for (EntryPoint e : p.getEntryPoints()) {
					elements.add(e);
					l.add("Task: "+e.getTask().getName());
					
				}
				if (p instanceof Plan) {
					Plan pp = (Plan)p;
					for (State s : pp.getStates()) {
						elements.add(s);
						l.add("State: "+s.getName());
					}
				}
				values = (String[]) l.toArray(new String[l.size()]);
				return new ComboBoxCellEditor((Composite)getViewer().getControl(),values);
				//return new TextCellEditor((Composite)getViewer().getControl());
			}
			
			@Override
			protected boolean canEdit(Object element) {
				return true;
			}
		});
		TableViewerColumn let = new TableViewerColumn(table,SWT.CENTER);
		let.getColumn().setWidth(60);
		let.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return "let v be";
			}
		});
		TableViewerColumn sorts = new TableViewerColumn(table,SWT.NONE);
		sorts.getColumn().setWidth(160);
		sorts.getColumn().setResizable(true);
		sorts.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				if (element == null) return "";
				String ret = "";
				GuiQuantifier q = (GuiQuantifier)element;
				for(int i=0; i< q.sorts.size(); i++) {
					if (i>0) ret +=", ";
					ret += q.sorts.get(i);
				}
				return ret;
			}
		});
		sorts.setEditingSupport(new EditingSupport(table) {
			
			@Override
			protected void setValue(final Object element, Object value) {
				String s = (String)value;
				final String[] ss = s.split(",");
				
				GuiQuantifier gq = ((GuiQuantifier)element);
				gq.sorts.clear();
				for (String string : ss) {
					string = string.trim();
					if(!string.isEmpty()) gq.sorts.add(string);
				}				
				getViewer().update(element,null);
				
			}
			
			@Override
			protected Object getValue(Object element) {
				if (element == null) return "";
				String ret = "";
				GuiQuantifier q = (GuiQuantifier)element;
				for(int i=0; i< q.sorts.size(); i++) {
					if (i>0) ret +=", ";
					ret += q.sorts.get(i);
				}
				return ret;
			}
			
			@Override
			protected CellEditor getCellEditor(Object element) {			
				return new TextCellEditor((Composite)getViewer().getControl());
			}
			
			@Override
			protected boolean canEdit(Object element) {
				return true;
			}
		});
		TableViewerColumn delbut = new TableViewerColumn(table,SWT.RIGHT);
		delbut.getColumn().setWidth(50);
		delbut.getColumn().setResizable(false);
		delbut.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public Image getImage(Object element) {
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_DELETE_EDIT);
			}
			@Override
			public String getText(Object element) {
				
				return "";
			}
			
		});
		delbut.setEditingSupport(new EditingSupport(table) {
			
			@Override
			protected void setValue(final Object element, Object value) {
				
				quantifiers.remove(element);
				inputChanged();
				//getViewer().update(element,null);
				
			}
			
			@Override
			protected Object getValue(Object element) {
				// TODO Auto-generated method stub
				return true;
			}
			
			@Override
			protected CellEditor getCellEditor(Object element) {
				return new CheckboxCellEditor(null, SWT.PUSH);
			}
			
			@Override
			protected boolean canEdit(Object element) {
				return true;
			}
		});
		
		
		table.getTable().setLinesVisible(true);
		data = new FormData();
		data.top = new FormAttachment(addButton,0, SWT.DEFAULT);
		data.bottom = new FormAttachment(100,0);
		data.right = new FormAttachment(100,0);
		data.left = new FormAttachment(0,0);
		table.getControl().setLayoutData(data);
		
		table.setInput(this.quantifiers);	
		setControl(createComposite);
		fillInput();
	}
	private void fillInput() {
		if(this.condition != null){
			EList<Quantifier> qs = this.condition.getQuantifiers();
			
			for(Quantifier q: qs) {
				GuiQuantifier gq = new GuiQuantifier(q.getScope());
				for(String s : q.getSorts()) {
					gq.sorts.add(s);
				}
				this.quantifiers.add(gq);
			}
		}
		inputChanged();
		
	}
	private void inputChanged() {
		table.refresh();
	}
	public ArrayList<GuiQuantifier> GetResult() {
		return this.quantifiers;
	}

}
