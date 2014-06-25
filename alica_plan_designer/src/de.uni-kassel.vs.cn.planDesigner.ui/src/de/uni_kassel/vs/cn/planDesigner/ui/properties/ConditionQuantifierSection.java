package de.uni_kassel.vs.cn.planDesigner.ui.properties;

import java.util.ArrayList;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.jface.viewers.CellEditor;
import org.eclipse.jface.viewers.CheckboxCellEditor;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.ComboBoxCellEditor;
import org.eclipse.jface.viewers.EditingSupport;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.jface.viewers.TextCellEditor;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.ForallAgents;
import de.uni_kassel.vs.cn.planDesigner.alica.IInhabitable;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.Quantifier;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

public class ConditionQuantifierSection extends PMLPropertySection {
	private TableViewer table;
	private Button addButton;

	@Override
	public void createControls(Composite parent, TabbedPropertySheetPage tabbedPropertySheetPage) {
		super.createControls(parent, tabbedPropertySheetPage);

		parent.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));
		Composite createComposite = getWidgetFactory().createFlatFormComposite(parent);
		addButton = getWidgetFactory().createButton(createComposite, "Add", SWT.PUSH);
		FormData data = new FormData();
		data.left = new FormAttachment(0, 0);
		addButton.setLayoutData(data);
		addButton.addListener(SWT.Selection, getEditController());

		table = new TableViewer(createComposite, SWT.BORDER | SWT.FULL_SELECTION);

		table.setContentProvider(new IStructuredContentProvider() {

			public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
				// TODO Auto-generated method stub

			}

			public void dispose() {
				// TODO Auto-generated method stub

			}

			public Object[] getElements(Object inputElement) {
				Condition c;
				if (inputElement instanceof Transition) {
					c = ((Transition) inputElement).getPreCondition();
				} else {
					c = (Condition) inputElement;
				}
				Object[] ret = new Object[c.getQuantifiers().size()];
				int i = 0;
				for (; i < c.getQuantifiers().size(); i++) {
					ret[i] = c.getQuantifiers().get(i);
				}
				return ret;
			}
		});

		TableViewerColumn type = new TableViewerColumn(table, SWT.LEFT);
		type.getColumn().setWidth(120);
		// type.getColumn().setText("Name");

		// TableViewerColumn sorts = new TableViewerColumn(table,SWT.NONE);
		// TableViewerColumn but = new TableViewerColumn(table,SWT.NONE);

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

				return new ComboBoxCellEditor((Composite) getViewer().getControl(), values);
				// return new
				// TextCellEditor((Composite)getViewer().getControl());
			}

			@Override
			protected boolean canEdit(Object element) {
				return true;
			}
		});
		TableViewerColumn in = new TableViewerColumn(table, SWT.CENTER);
		in.getColumn().setWidth(25);
		in.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return "in";
			}
		});
		TableViewerColumn scope = new TableViewerColumn(table, SWT.NONE);
		scope.getColumn().setWidth(120);
		scope.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				Quantifier q = (Quantifier) element;
				if (q.getScope() == null)
					return "";
				if (q.getScope() instanceof EntryPoint) {
					return ((EntryPoint) q.getScope()).getTask().getName();
				}
				return q.getScope().getName();
			}
		});

		scope.setEditingSupport(new EditingSupport(table) {
			ArrayList<IInhabitable> elements;
			String[] values;

			@Override
			protected void setValue(final Object element, Object value) {
				Integer pos = (Integer) value;
				if (pos >= 0 && pos < elements.size()) {
					final IInhabitable i = elements.get(pos);
					executeCommand(new RecordingCommand(getEditingDomain(), "Set Scope") {
						@Override
						protected void doExecute() {

							((Quantifier) element).setScope(i);

						}
					});
				} else {
					executeCommand(new RecordingCommand(getEditingDomain(), "Set Scope") {
						@Override
						protected void doExecute() {

							((Quantifier) element).setScope(null);

						}
					});
				}
				getViewer().update(element, null);
			}

			@Override
			protected Object getValue(Object element) {
				return elements.indexOf(element);

			}

			@Override
			protected CellEditor getCellEditor(Object element) {
				elements = new ArrayList<IInhabitable>();
				// AbstractPlan p = ((Condition)getModel()).getAbstractPlan();
				Plan p = getPlan();
				ArrayList<String> l = new ArrayList<String>();
				elements.add(p);
				l.add("Plan: " + p.getName());
				for (EntryPoint e : p.getEntryPoints()) {
					elements.add(e);
					l.add("Task: " + e.getTask().getName());

				}
				if (p instanceof Plan) {
					Plan pp = (Plan) p;
					for (State s : pp.getStates()) {
						elements.add(s);
						l.add("State: " + s.getName());
					}
				}
				values = (String[]) l.toArray(new String[l.size()]);
				return new ComboBoxCellEditor((Composite) getViewer().getControl(), values);
				// return new
				// TextCellEditor((Composite)getViewer().getControl());
			}

			@Override
			protected boolean canEdit(Object element) {
				return true;
			}
		});
		TableViewerColumn let = new TableViewerColumn(table, SWT.CENTER);
		let.getColumn().setWidth(60);
		let.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				return "let v\u20D7 be";
			}
		});
		TableViewerColumn sorts = new TableViewerColumn(table, SWT.NONE);
		sorts.getColumn().setWidth(160);
		sorts.getColumn().setResizable(true);
		sorts.setLabelProvider(new ColumnLabelProvider() {
			@Override
			public String getText(Object element) {
				if (element == null)
					return "";
				String ret = "";
				Quantifier q = (Quantifier) element;
				for (int i = 0; i < q.getSorts().size(); i++) {
					if (i > 0)
						ret += ", ";
					ret += q.getSorts().get(i);
				}
				return ret;
			}
		});
		sorts.setEditingSupport(new EditingSupport(table) {

			@Override
			protected void setValue(final Object element, Object value) {
				String s = (String) value;
				final String[] ss = s.split(",");

				executeCommand(new RecordingCommand(getEditingDomain(), "Set Scope") {
					@Override
					protected void doExecute() {
						Quantifier q = (Quantifier) element;
						EList<String> l = q.getSorts();
						l.clear();
						for (String string : ss) {
							string = string.trim();
							if (!string.isEmpty())
								l.add(string);
						}

					}
				});

				getViewer().update(element, null);

			}

			@Override
			protected Object getValue(Object element) {
				if (element == null)
					return "";
				String ret = "";
				Quantifier q = (Quantifier) element;
				for (int i = 0; i < q.getSorts().size(); i++) {
					if (i > 0)
						ret += ", ";
					ret += q.getSorts().get(i);
				}
				return ret;
			}

			@Override
			protected CellEditor getCellEditor(Object element) {
				return new TextCellEditor((Composite) getViewer().getControl());
			}

			@Override
			protected boolean canEdit(Object element) {
				return true;
			}
		});
		TableViewerColumn delbut = new TableViewerColumn(table, SWT.RIGHT);
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
				executeCommand(new RecordingCommand(getEditingDomain(), "Delete Quantifier") {
					@Override
					protected void doExecute() {

						getCondition().getQuantifiers().remove(element);
					}
				});

				getViewer().update(element, null);

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

		/*
		 * Button but = new Button(table.getTable(), SWT.PUSH); but.pack();
		 * but.setText("x");
		 */

		table.getTable().setLinesVisible(true);
		data = new FormData();
		data.top = new FormAttachment(addButton, 0, SWT.DEFAULT);
		data.bottom = new FormAttachment(100, 0);
		data.right = new FormAttachment(100, 0);
		data.left = new FormAttachment(0, 0);
		table.getControl().setLayoutData(data);

		// table.getTable().setHeaderVisible(true);
	}

	private Condition getCondition() {
		EObject selection = getModel();
		if (selection instanceof Transition) {
			return ((Transition) selection).getPreCondition();
		} else {
			// we know that only a Condition can be selected here
			return ((Condition) selection);
		}
	}

	protected Plan getPlan() {
		return (Plan) getModel().eContainer();
	}
	
	@Override
	public void refresh() {
		getEditController().updateView(null);
	}

	@Override
	protected void selectionEvent(Object source) {
		if (source.equals(addButton)) {

			executeCommand(new RecordingCommand(getEditingDomain(), "Add Quantifier") {
				@Override
				protected void doExecute() {
					ForallAgents fa = AlicaFactory.eINSTANCE.createForallAgents();

					getCondition().getQuantifiers().add(fa);
				}
			});

			table.setSelection(new StructuredSelection(table.getTable().getItemCount() - 1), true);

		}
	}

	@Override
	protected void updateView(Notification n) {
		final Condition condition = getCondition();

		if (condition != null) {
			table.setInput(condition);
		}

		table.refresh();
	}

}
