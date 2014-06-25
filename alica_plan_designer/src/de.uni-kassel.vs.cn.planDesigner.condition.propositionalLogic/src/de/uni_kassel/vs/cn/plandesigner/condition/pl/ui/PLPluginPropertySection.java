package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.util.EMap;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.ModifyEvent;
import org.eclipse.swt.events.ModifyListener;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.List;
import org.eclipse.swt.widgets.Text;
import org.eclipse.ui.IWorkbenchPart;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.PMLPropertySection;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.automat.ValidationResult;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Formular;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.PropositionalLogicPlugin;

public class PLPluginPropertySection extends PMLPropertySection {
	private Text inputText;
	private List list;
	private Label statusLabel;
	private HandleFormularsComposite handleFormularsComposite;

	@Override
	public void createControls(Composite parent, TabbedPropertySheetPage tabbedPropertySheetPage) {
		super.createControls(parent, tabbedPropertySheetPage);

		
		handleFormularsComposite = new HandleFormularsComposite(parent, SWT.NONE, PropositionalLogicPlugin.getInstance().getCompleteVocabulary());
		handleFormularsComposite.getText().addModifyListener(new SaveFormularListener());

	

	}

	@Override
	public void setInput(IWorkbenchPart part, ISelection selection) {
		super.setInput(part, selection);

		// after setInput, condition is available
		Condition condition = getCondition();

		String formular = (String) condition.getParameters().get(PluginConstants.PARAMETER_FORMULAR);
		if (formular != null) {
			handleFormularsComposite.getText().append(formular);

			// check if formular is valid.
			ValidationResult validationResult = handleFormularsComposite.isFormularValid();
			handleFormularsComposite.setStatus(validationResult.getStatusMessage(), validationResult.isValid());
		} else {
			handleFormularsComposite.setStatus(PluginConstants.STATUS_MISSING_OPERAND, false);
		}
	}

	public Condition getCondition() {
		EObject model = getModel();
		if (model instanceof Transition) {
			return ((Transition) model).getPreCondition();
		} else {
			return (Condition) model;
		}
	}

	@Override
	protected void updateView(Notification n) {
		System.out.println("PLPluginPropertySection.updateView()");
	}

	/**
	 * Saves the formular in the condtion.
	 */
	private void saveFormular() {
		getCommandStack().execute(new RecordingCommand(getEditingDomain()) {

			@Override
			protected void doExecute() {
				Formular formular = handleFormularsComposite.getFormular();
				EMap<String, Object> pluginParameters = getCondition().getParameters();

				pluginParameters.put(PluginConstants.PARAMETER_FORMULAR, formular.getExpression());
				pluginParameters.put(PluginConstants.PARAMETER_RESOLVED_FORMULAR, formular.getResolvedExpression());
				pluginParameters.put(PluginConstants.PARAMETER_RESOLVED_OPERANDS, formular.getOperands());
			}
		});
	}

	public Text getInputText() {
		return inputText;
	}

	public void setInputText(Text text) {
		this.inputText = text;
	}

	public List getList() {
		return list;
	}

	public void setList(List list) {
		this.list = list;
	}

	public Label getStatusLabel() {
		return statusLabel;
	}

	public void setStatusLabel(Label statusLabel) {
		this.statusLabel = statusLabel;

	}



	/**
	 * After every input formular is add to pluginParameters in Condition
	 * 
	 * @author philipp
	 * 
	 */
	private class SaveFormularListener implements ModifyListener {

		@Override
		public void modifyText(ModifyEvent e) {
			saveFormular();

			// validate formular and set status in ui, this is done from
			// outside, because components which uses the
			// HandleFormularComposite may also want to set statusmessages.
			ValidationResult result = handleFormularsComposite.isFormularValid();
			handleFormularsComposite.setStatus(result.getStatusMessage(), result.isValid());
		}

	}
}
