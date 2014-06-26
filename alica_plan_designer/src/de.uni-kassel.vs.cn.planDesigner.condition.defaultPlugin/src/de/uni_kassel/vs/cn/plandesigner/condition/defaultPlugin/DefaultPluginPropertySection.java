package de.uni_kassel.vs.cn.plandesigner.condition.defaultPlugin;

import java.util.Collections;

import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Widget;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PreCondition;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.PMLPropertySection;

public class DefaultPluginPropertySection extends PMLPropertySection {

	private Button enabledButton;
	private Text nameText;
	private Text conditionText;
	private Label nameLabel;
	private Label conditionLabel;
	private Shell shell;

	@Override
	public void createControls(Composite parent, TabbedPropertySheetPage tabbedPropertySheetPage) {
		super.createControls(parent, tabbedPropertySheetPage);

		Composite form = getWidgetFactory().createFlatFormComposite(parent);

		enabledButton = getWidgetFactory().createButton(form, "Enabled", SWT.CHECK);

		nameLabel = getWidgetFactory().createLabel(form, "Name:");
		nameText = getWidgetFactory().createText(form, "", SWT.BORDER | SWT.SINGLE);

		conditionLabel = getWidgetFactory().createLabel(form, "Condition:");
		conditionText = getWidgetFactory().createText(form, "", SWT.BORDER | SWT.MULTI | SWT.V_SCROLL);

		// Do the layout
		FormData data = null;

		data = new FormData();
		data.top = new FormAttachment(0, 0);
		data.left = new FormAttachment(0, 0);

		enabledButton.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(enabledButton, 0);
		data.left = new FormAttachment(0, 0);
		nameLabel.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(enabledButton, 0);
		data.left = new FormAttachment(enabledButton, 0);
		data.right = new FormAttachment(50, 0);
		nameText.setLayoutData(data);

		data = new FormData(SWT.DEFAULT, 100);
		data.top = new FormAttachment(nameText, 0);
		data.left = new FormAttachment(enabledButton, 0);
		data.right = new FormAttachment(50, 0);
		conditionText.setLayoutData(data);

		registerListeners();
		// refreshControls();

		form.layout();
		form.pack();

	}

	@Override
	protected void basicSetInput(Object input) {
		super.basicSetInput(input);
		// Reset the UI
		getEditController().pauseListening();
		resetUI();
		getEditController().resumeListening();

		refreshControls();

	}

	@Override
	protected void addAllAdapters() {
		super.addAllAdapters();
		// Add an adapter to the condition if possible
		if (getCondition() != null)
			getEditController().addToObject(getCondition());
	}

	private void registerListeners() {
		getEnabledButton().addListener(SWT.Selection, getEditController());
		getNameText().addListener(SWT.KeyDown, getEditController());
		getNameText().addListener(SWT.FocusOut, getEditController());
		getConditionText().addListener(SWT.FocusOut, getEditController());
	}

	protected Button getEnabledButton() {
		return enabledButton;
	}

	protected Text getNameText() {
		return nameText;
	}

	protected Text getConditionText() {
		return conditionText;
	}

	protected Label getNameLabel() {
		return nameLabel;
	}

	protected Label getConditionLabel() {
		return conditionLabel;
	}

	/**
	 * Enabled or disables all visuals depending on the editable state
	 * 
	 * @param enabled
	 */
	protected void refreshControls() {
		boolean editable = isEditable();

		getEnabledButton().setEnabled(editable);

		Condition condition = getCondition();

		boolean isEnabled = false;
		if (condition instanceof PreCondition) {
			isEnabled = ((PreCondition) condition).isEnabled();
			getEnabledButton().setSelection(isEnabled);
		} else {
			// boolean enabledRadioSelected = (condition != null &&
			// getEnabledButton().getSelection());
			isEnabled = condition != null;
		}
		getEnabledButton().setSelection(isEnabled);

		getNameLabel().setEnabled(editable && isEnabled);
		getNameText().setEnabled(editable && isEnabled);
		getConditionLabel().setEnabled(editable && isEnabled);
		getConditionText().setEnabled(editable && isEnabled);

		if (condition != null) {
			getNameText().setText(condition.getName());
			getConditionText().setText(condition.getConditionString());
		}

	}

	@Override
	protected void updateView(Notification n) {
		final Condition condition = getCondition();
		if (condition != null) {
			try {
				getEditingDomain().runExclusive(new Runnable() {
					public void run() {
						// getEnabledButton().setSelection(condition.isEnabled());
						getNameText().setText(condition.getName());
						getConditionText().setText(condition.getConditionString());
						// EList<Variable> vars = condition.getVars();
						// StringBuffer buf = new StringBuffer();
						// for (Variable v : vars) {
						// buf.append(v.getName());
						// buf.append(", ");
						// }
						// getVariableText().setText(buf.substring(0,
						// Math.max(buf.length() - 2, 0)));
						//
						// buf = new StringBuffer();
						// for (Quantifier q : condition.getQuantifiers()) {
						// if (q instanceof ForallAgents) {
						// String sname = q.getScope().getName();
						// if (q.getScope() instanceof EntryPoint) {
						// sname = ((EntryPoint)
						// q.getScope()).getTask().getName();
						// }
						// buf.append("Forall Agents in " + sname +
						// " let v be (");
						// boolean first = true;
						// for (String s : q.getSorts()) {
						// if (first) {
						// buf.append(s);
						// first = false;
						// } else
						// buf.append(", " + s);
						// }
						// buf.append(")\n");
						// } else {
						// buf.append("Unknown Quantifier!\n");
						// }
						// }
						// getQuantText().setText(buf.toString());

					}
				});
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}
		refreshControls();
	}

	protected void resetUI() {
		getNameText().setText("");
		getConditionText().setText("");
	}

	@Override
	public void refresh() {
		getEditController().updateView(null);
	}

	@Override
	protected void selectionEvent(Object source) {
		if (source.equals(getEnabledButton())) {
			boolean enabled = getEnabledButton().getSelection();

			// enableVisuals(enabled);
			Condition condition = getCondition();
			CompoundCommand cmd = new CompoundCommand(0);
			if (condition == null) {
				condition = (PreCondition) AlicaFactory.eINSTANCE.create(AlicaPackage.eINSTANCE.getPreCondition());
				cmd.append(CreateChildCommand.create(getEditingDomain(), getModel(), new CommandParameter(getModel(), AlicaPackage.eINSTANCE.getTransition_PreCondition(), condition),
						Collections.EMPTY_LIST));

				// Add the model to the editController
				getEditController().addToObject(condition);
			}

			cmd.append(SetCommand.create(getEditingDomain(), condition, AlicaPackage.eINSTANCE.getPreCondition_Enabled(), enabled));

			executeCommand(cmd);
			refreshControls();
		}
	}

	//
	@Override
	protected void focusOutEvent(Widget source) {
		applyValueToModel(source);
	}

	@Override
	protected void enterPressed(Widget source) {
		// Save caret position
		int pos = ((Text) source).getCaretPosition();
		applyValueToModel(source);
		// Apply the caret position
		((Text) source).setSelection(pos);
	}

	//
	private void applyValueToModel(Widget source) {
		System.out.println("DefaultPluginPropertySection.applyValueToModel()");
		if (source.equals(getNameText())) {
			System.out.println("DefaultPluginPropertySection.applyValueToModel(): equals name text");
			if (getCondition() != null) {
				System.out.println("DefaultPluginPropertySection.applyValueToModel(): condtion not null");
				executeCommand(SetCommand.create(getEditingDomain(), getCondition(), AlicaPackage.eINSTANCE.getPlanElement_Name(), getNameText().getText()));
			}
		} else if (source.equals(getConditionText())) {
			if (getCondition() != null) {
				executeCommand(SetCommand.create(getEditingDomain(), getCondition(), AlicaPackage.eINSTANCE.getCondition_ConditionString(), getConditionText().getText()));
			}
		}
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

	protected EClass getConditionType() {
		// Return the PreCondition MetaClass
		return AlicaPackage.eINSTANCE.getPreCondition();
	}

}
