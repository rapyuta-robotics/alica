package de.uni_kassel.vs.cn.planDesigner.ui.properties;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.jface.wizard.WizardDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.alica.Variable;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLModifyConditionVariablesWizard;

public class ConditionVariablesSection extends PMLPropertySection {
	private Text varText;
	private Button modifyVarsButton;
	private Shell shell;

	@Override
	public void createControls(Composite parent, TabbedPropertySheetPage tabbedPropertySheetPage) {
		super.createControls(parent, tabbedPropertySheetPage);
		this.shell = parent.getShell();
		Group group = getWidgetFactory().createGroup(parent, "Variables");
		group.setLayout(new FillLayout());
		Composite form = getWidgetFactory().createFlatFormComposite(group);

		varText = getWidgetFactory().createText(form, "", SWT.BORDER | SWT.MULTI | SWT.WRAP | SWT.V_SCROLL);
		modifyVarsButton = getWidgetFactory().createButton(form, "Modify", SWT.DEFAULT);

		// Do the layout
		FormData data = null;

		data = new FormData(SWT.DEFAULT, 50);
		data.top = new FormAttachment(0, 0);
		data.left = new FormAttachment(0, 0);
		data.right = new FormAttachment(50, 0);
		varText.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(varText, 0);
		data.left = new FormAttachment(0, 0);
		modifyVarsButton.setLayoutData(data);

		registerListeners();
	}

	@Override
	protected void basicSetInput(Object input) {
		super.basicSetInput(input);

		// Reset the UI
		getEditController().pauseListening();
		resetUI();
		getEditController().resumeListening();
	}

	private void registerListeners() {
		getVariableButton().addListener(SWT.Selection, getEditController());
	}

	protected Button getVariableButton() {
		return modifyVarsButton;
	}

	protected Text getVariableText() {
		return varText;
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

	@Override
	protected void updateView(Notification n) {
		final Condition condition = getCondition();

		if (condition != null) {

			try {
				getEditingDomain().runExclusive(new Runnable() {
					public void run() {
						EList<Variable> vars = condition.getVars();
						StringBuffer buf = new StringBuffer();
						for (Variable v : vars) {
							buf.append(v.getName());
							buf.append(", ");
						}
						getVariableText().setText(buf.substring(0, Math.max(buf.length() - 2, 0)));
					}
				});
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

		}
	}

	protected void resetUI() {
		getVariableText().setText("");
	}

	@Override
	public void refresh() {
		getEditController().updateView(null);
	}

	@Override
	protected void selectionEvent(Object source) {
		if (source.equals(getVariableButton())) {
			Condition c = getCondition();
			PMLModifyConditionVariablesWizard wiz = new PMLModifyConditionVariablesWizard(c, getPlan());

			WizardDialog dialog = new WizardDialog(shell, wiz);
			// getViewer().getControl().getShell(), wiz);

			dialog.setBlockOnOpen(true);
			dialog.open();
		}
	}

	protected Plan getPlan() {
		return (Plan) getModel().eContainer();
	}

}
