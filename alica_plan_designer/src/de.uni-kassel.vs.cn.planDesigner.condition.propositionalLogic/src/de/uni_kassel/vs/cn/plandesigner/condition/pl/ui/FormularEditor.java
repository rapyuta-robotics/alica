package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.dialogs.IDialogConstants;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.jface.viewers.TreeNode;
import org.eclipse.jface.viewers.TreeSelection;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.KeyListener;
import org.eclipse.swt.events.ModifyEvent;
import org.eclipse.swt.events.ModifyListener;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.events.MouseMoveListener;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Menu;
import org.eclipse.swt.widgets.MenuItem;
import org.eclipse.swt.widgets.MessageBox;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;

import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.ui.util.UsageDialog;
import de.uni_kassel.vs.cn.plandesigner.condition.core.Util;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.automat.ValidationResult;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Formular;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Proposition;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.PropositionalLogicPlugin;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Vocabulary;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider.FormularImageProvider;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider.FormularTextProvider;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider.StringTextProvider;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider.TreeLabelProvider;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider.TreeNodeContentProvider;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.utils.FormularWorker;

public class FormularEditor extends Dialog{
	/**
	 * Textfield for the name of new formular
	 */
	private Text nameText;

	private HandleFormularsComposite formularComposite;

	// private List formularList;
	private TreeViewer formularViewer;

	/**
	 * Old formular, select by the list
	 */
	private Formular oldFormular;

	private Button addButton;
		
	protected FormularEditor(Shell parentShell) {
		super(parentShell);
	}
		

	@Override
	protected Control createDialogArea(Composite parent) {

		// create ui
		Composite container = (Composite) super.createDialogArea(parent);

		container.setLayout(new GridLayout(2, false));
		Label label = new Label(container, SWT.NONE);
		label.setText(PluginConstants.HEADLINE_REPOSITORY);

		label = new Label(container, SWT.NONE);
		label.setText(PluginConstants.HEADLINE_PROPERTIES);

		formularViewer = new TreeViewer(container, SWT.BORDER | SWT.V_SCROLL | SWT.H_SCROLL);
		formularViewer.getControl().setLayoutData(new GridData(SWT.LEFT, SWT.FILL, false, true, 1, 1));
		formularViewer.setContentProvider(new TreeNodeContentProvider());

		TreeLabelProvider labelProvider = new TreeLabelProvider(new FormularTextProvider(), new FormularImageProvider());
		formularViewer.setLabelProvider(labelProvider);
		formularViewer.setInput(PropositionalLogicPlugin.getInstance().getFormulaTree());

		Menu menu = new Menu(formularViewer.getTree());
		MenuItem item = new MenuItem(menu, SWT.NONE);
		item.setText(PluginConstants.UI_SHOW_USAGE);
		formularViewer.getTree().setMenu(menu);

		Composite editComposite = new Composite(container, SWT.NONE);
		editComposite.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true, 1, 1));
		editComposite.setLayout(new GridLayout(1, true));

		Label labelName = new Label(editComposite, SWT.NONE);
		labelName.setText("Name");

		nameText = new Text(editComposite, SWT.BORDER);
		nameText.setSize(800, nameText.getSize().y);
		nameText.setLayoutData(new GridData(SWT.FILL, SWT.TOP, true, false, 1, 1));

		formularComposite = new HandleFormularsComposite(editComposite, SWT.NONE, PropositionalLogicPlugin.getInstance().getCompleteVocabulary());
		formularComposite.setLayoutData(new GridData(SWT.FILL, SWT.CENTER, true, false, 1, 1));
		
		formularComposite.getOpenFormularEditorButton().setVisible(false);

		Composite buttonComposite = new Composite(editComposite, SWT.NONE);
		buttonComposite.setLayoutData(new GridData(SWT.FILL, SWT.CENTER, true, false, 1, 1));
		buttonComposite.setLayout(new GridLayout(4, false));

		addButton = new Button(buttonComposite, SWT.PUSH);
		addButton.setText(PluginConstants.UI_ADD);

		Button removeButton = new Button(buttonComposite, SWT.PUSH);
		removeButton.setText(PluginConstants.UI_REMOVE);

		Button newButton = new Button(buttonComposite, SWT.PUSH);
		newButton.setText(PluginConstants.UI_NEW);

		Button validateButton = new Button(buttonComposite, SWT.PUSH);
		validateButton.setText(PluginConstants.UI_VALIDATE);

		// set listener
		formularViewer.addSelectionChangedListener(new SelectFormularListener());
		addButton.addSelectionListener(new AddFormularSelectionListener());
		removeButton.addSelectionListener(new RemoveSelectionListener());
		newButton.addSelectionListener(new NewSelectionListener());
		validateButton.addSelectionListener(new ValidateFormularsListener());
		item.addSelectionListener(new ShowUsageSelectionListener());
		formularComposite.getText().addModifyListener(new UpdateStatusListener());
		nameText.addModifyListener(new UpdateStatusListener());

		updateStatus();
		// set size
						
		return container;

	}
	
	@Override
	protected void createButtonsForButtonBar(Composite parent) {
		createButton(parent, IDialogConstants.OK_ID,IDialogConstants.OK_LABEL, false);
	}

	@Override
	protected boolean isResizable() {
		return true;
	}

	@Override
	protected void configureShell(Shell newShell) {
		// Overwritten to set the title of the dialog
		super.configureShell(newShell);
		newShell.setText(PluginConstants.UI_TITLE_FORMULAR_EDITOR);	
	}

	/**
	 * Checks if formular is valid and adds it to the vocabulary.s
	 * 
	 * @return true if formular was valid and added to the vocabulary, false
	 *         otherwise.
	 */

	private boolean checkAndAddFormular() {
		// does the new expression contains itself
		String expression = getFormularComposite().getText().getText();
		MessageBox mb = new MessageBox(getShell());
		mb.setText(PluginConstants.HEADLINE_WRONG_FORMULAR);

		if ((expression != null) && (oldFormular != null) && (expression.contains(oldFormular.getName()))) {
			mb.setMessage(PluginConstants.STATUS_FORBIDDEN_CIRCLE);
			mb.open();
			return false;
		}

		Formular formular = buildFormularFromInput();
		if ("".equals(formular.getName())) {
			mb.setMessage(PluginConstants.STATUS_MISSING_NAME);
			mb.open();
			return false;
		} else if ("".equals(formular.getExpression()) || " ".equals(formular.getExpression())) {
			mb.setMessage(PluginConstants.STATUS_MISSING_EXPRESSION);
			mb.open();
			return false;
		} else if (!formular.isValid().isValid()) {
			mb.setMessage(PluginConstants.STATUS_FORMULAR_NOT_VALID);
			mb.open();
			return false;
		} else {
			if (oldFormular != null) {
				// was old formular renamed?
				if (!oldFormular.getName().equals(formular.getName())) {
					// is name duplicated?
					if (PropositionalLogicPlugin.getInstance().getCompleteVocabulary().contains(formular.getName())) {
						int result = showOverrideMessageBox();
						if (result == 1) {
							// user don't want to override the formula
							return false;
						}
					}
				}

				Vocabulary.removeFromCorrectVocabulary(oldFormular);
				addFormular(formular);

				// rename in all expressions to new formular
				Vocabulary voc = PropositionalLogicPlugin.getInstance().getLinkedFormularsVocabulary();
				String oldName = oldFormular.getName();
				String newName = formular.getName();
				for (String name : voc.getFormularNames()) {
					Formular f = voc.getFormular(name);
					String oldExpression = f.getExpression();
					String newExpression = oldExpression.replaceAll(oldName, newName);

					f.setExpression(newExpression);
				}

				return true;
			} else {
				// is name duplicated?
				if (PropositionalLogicPlugin.getInstance().getCompleteVocabulary().contains(formular.getName())) {
					int result = showOverrideMessageBox();
					if (result == 1) {
						// user don't want to override the formula
						return false;
					} else {
						// remove the old formular
						Vocabulary.removeFromCorrectVocabulary(formular);
					}
				}

				// everything finde with new formular, just add the new formular
				addFormular(formular);
				return true;
			}
		}
	}

	/**
	 * Returns formular instance with elements given by input from the user.
	 * 
	 * @return
	 */
	private Formular buildFormularFromInput() {
		String name = getNameText().getText();
		Formular formular;
		boolean isProposition = isProposition(formularComposite.getText().getText());

		if (isProposition) {
			formular = formularComposite.getProposition();
		} else {
			formular = formularComposite.getFormular();
		}

		formular.setName(name);
		formular.addChilds();

		return formular;
	}

	/**
	 * Show notification dialog, that Formular will be overriden
	 * 
	 * @return
	 */
	private int showOverrideMessageBox() {
		// no it isn't ask for overriding the old formular
		String[] labels = new String[2];
		labels[0] = "Yes";
		labels[1] = "No";
		MessageDialog dialog = new MessageDialog(getShell(), "Override Formular", null, PluginConstants.STATUS_OVERRIDE_FORMULAR, SWT.NONE, labels, 1);
		int result = dialog.open();
		return result;
	}

	/**
	 * Updates the tree inside the list of formulars.
	 */
	private void updateTree() {
		List<TreeNode> formulaTree = PropositionalLogicPlugin.getInstance().getFormulaTree();

		formularViewer.setInput(formulaTree);
		formularViewer.getControl().redraw();
		formularViewer.getControl().update();
	}

	/**
	 * Resets the ui to initial state
	 */
	public void reset() {
		oldFormular = null;
		addButton.setText(PluginConstants.UI_ADD);
		getNameText().setText("");
		getFormularComposite().getText().setText("");
		getFormularComposite().setStatus("", true);
	}

	/**
	 * Validates all formulars in the repositiory
	 * 
	 * @author philipp
	 * 
	 */
	private class ValidateFormularsListener implements SelectionListener {

		@Override
		public void widgetSelected(SelectionEvent e) {
			Map<Formular, List<String>> errors = PropositionalLogicPlugin.getInstance().validateFormulars();

			List<TreeNode> nodes = new ArrayList<TreeNode>();

			for (Formular f : errors.keySet()) {
				TreeNode errorRoot = new TreeNode(f.getName());
				List<String> errorMessages = errors.get(f);

				TreeNode[] errorChilds = new TreeNode[errorMessages.size()];
				for (int i = 0; i < errorChilds.length; ++i) {
					String error = errorMessages.get(i);
					TreeNode node = new TreeNode(error);
					errorChilds[i] = node;
				}

				if (errorChilds.length > 0) {
					errorRoot.setChildren(errorChilds);
					nodes.add(errorRoot);
				}

			}

			if (nodes.isEmpty()) {
				MessageBox mb = new MessageBox(getShell());
				mb.setMessage(PluginConstants.STATUS_EVERY_FORMULAR_VALID);
				mb.open();
			} else {
				// Show invalid formulars
				TreeLabelProvider provider = new TreeLabelProvider(new StringTextProvider(), new FormularImageProvider());
				TreeNodeContentProvider contentProvider = new TreeNodeContentProvider();
				TreeDialog dialog = new TreeDialog(getShell(), contentProvider, provider, nodes);
				dialog.open();

			}
		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {
			// NOTHING TO DO

		}

	}

	/**
	 * Listener which initializes the ui after user choose a formular from the
	 * treeviewer.
	 * 
	 * @author philipp
	 * 
	 */
	private class SelectFormularListener implements ISelectionChangedListener {

		@Override
		public void selectionChanged(SelectionChangedEvent event) {
			TreeSelection selection = (TreeSelection) event.getSelection();
			TreeNode node = (TreeNode) selection.getFirstElement();
			if (node != null) {
				Formular f = (Formular) node.getValue();

				addButton.setText(PluginConstants.UI_EDIT);
				// init ui elements
				oldFormular = f;
				getNameText().setText(f.getName());
				formularComposite.getText().setText(f.getExpression());
				// TODO hier muss unterschied zwischen proposition und formel
				// gemacht werden
				updateStatus();

			} else {
				System.out.println("Node is null");
			}
		}
	}

	/**
	 * Listener to show usage of a formula
	 * 
	 * @author philipp
	 * 
	 */
	private class ShowUsageSelectionListener implements SelectionListener {

		@Override
		public void widgetSelected(SelectionEvent e) {
			TreeSelection selection = (TreeSelection) formularViewer.getSelection();
			if (selection != null) {
				// node from repository tree viewer
				TreeNode node = (TreeNode) selection.getFirstElement();
				Formular formular = (Formular) node.getValue();

				// nodes for the usage dialog
				List<TreeNode> treeNodes = new ArrayList<TreeNode>();

				Map<Plan, List<Condition>> allConditions = Util.getAllConditions();
				for (Plan plan : allConditions.keySet()) {
					TreeNode planNode = new TreeNode(plan);
					List<Condition> conditions = allConditions.get(plan);

					List<TreeNode> conditionNodes = new ArrayList<TreeNode>();

					for (Condition c : conditions) {
						String expression = (String) c.getParameters().get(PluginConstants.PARAMETER_FORMULAR);
						if (expression != null) {
							// to split expression to elements
							FormularWorker worker = new FormularWorker();
							List<String> elements = worker.splitToElements(expression);
							// save added elements (e.g. if formula is like A &
							// (!A | B))
							List<String> alreadyAdded = new ArrayList<String>();
							for (String element : elements) {
								if (alreadyAdded.contains(element)) {
									continue;
								}

								if (element.equals(formular.getName())) {
									TreeNode conditionNode = new TreeNode(c.getId());
									alreadyAdded.add(element);
									// check if node already in list
									conditionNodes.add(conditionNode);
								}
							}
						}
					}

					if (conditionNodes.size() > 0) {
						TreeNode[] planChilds = new TreeNode[conditionNodes.size()];
						for (int i = 0; i < planChilds.length; ++i) {
							planChilds[i] = conditionNodes.get(i);
						}

						planNode.setChildren(planChilds);
						treeNodes.add(planNode);
					}
				}

				UsageDialog dialog = new UsageDialog(getShell(), treeNodes, "Usage of formular", "Formular is used in the following plans: ", MessageDialog.INFORMATION);
				int result = dialog.open();
			}

		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {
			// NOTHING TO DO
		}
	}

	/**
	 * Selection Listener for adding a formular or proposition to the vocabulary
	 * 
	 * @author philipp
	 * 
	 */
	private class AddFormularSelectionListener implements SelectionListener {

		@Override
		public void widgetSelected(SelectionEvent e) {
			boolean success = checkAndAddFormular();
			if (success) {
				reset();
				updateTree();
				updateStatus();
			}
		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {
			// NOTHING TO DO
		}
	}

	/**
	 * Selection Listener for removing formulars and propositions from the
	 * vocabulary
	 * 
	 * @author philipp
	 * 
	 */
	private class RemoveSelectionListener implements SelectionListener {

		@Override
		public void widgetSelected(SelectionEvent e) {
			Formular formular = buildFormularFromInput();
			Vocabulary.removeFromCorrectVocabulary(formular);
			updateTree();
			reset();
			updateStatus();
		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {
			// NOTHING TO DO

		}

	}

	/**
	 * Selection Listener for resetting the ui to create a new formular
	 * 
	 * @author philipp
	 * 
	 */
	private class NewSelectionListener implements SelectionListener {

		@Override
		public void widgetSelected(SelectionEvent e) {
			reset();
			updateStatus();
			updateTree();
		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {
			// NOTHING TO DO
		}

	}

	/**
	 * Listener to decide if current input is a formular or a proposition. It is
	 * a formular if an operator appears in the textfield, otherwise it a
	 * proposition.
	 * 
	 * @author philipp
	 * 
	 */
	private class UpdateStatusListener implements ModifyListener {

		@Override
		public void modifyText(ModifyEvent e) {
			updateStatus();
		}

	}

	/**
	 * Updates the status.
	 */
	protected void updateStatus() {
		String input = formularComposite.getText().getText();
		if (formularComposite.getText().getText().equals("")) {
			formularComposite.setStatus(PluginConstants.STATUS_MISSING_EXPRESSION, false);
		} else if (getNameText().getText().equals("")) {
			formularComposite.setStatus(PluginConstants.STATUS_MISSING_NAME, false);
		} else if (isProposition(input)) {
			formularComposite.setStatus(PluginConstants.STATUS_PROPOSITION_IS_VALID, true);
		} else {
			// to set status in ui
			ValidationResult result = formularComposite.isFormularValid();
			formularComposite.setStatus(result.getStatusMessage(), result.isValid());
		}
	}

	/**
	 * Checks if the given input has syntax of formular or proposition.
	 * 
	 * @param input
	 * @return
	 */
	protected boolean isProposition(String input) {
		// proposition if input contains no operator and no whitespace
		if (input.contains(PluginConstants.OPERATOR_AND) || input.contains(PluginConstants.OPERATOR_OR) || input.contains(PluginConstants.OPERATOR_NEG)) {
			return false;
		} else {
			return true;
		}
	}

	protected void addFormular(Formular formular) {
		Vocabulary v;
		if (formular instanceof Proposition) {
			v = PropositionalLogicPlugin.getInstance().getPropositionVocabulary();
		} else {

			v = PropositionalLogicPlugin.getInstance().getLinkedFormularsVocabulary();
		}

		v.addToFormulars(formular);
		// save new vocabulary
		v.save();
	}

	protected Text getNameText() {
		return nameText;
	}

	protected HandleFormularsComposite getFormularComposite() {
		return formularComposite;
	}

}
