package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.util.ArrayList;
import java.util.Collections;
import java.util.regex.Pattern;

import org.eclipse.swt.SWT;
import org.eclipse.swt.events.FocusEvent;
import org.eclipse.swt.events.FocusListener;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.KeyListener;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.events.MouseListener;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.graphics.Device;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.List;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Text;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.automat.Automat;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.automat.ValidationResult;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Formular;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Proposition;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.PropositionalLogicPlugin;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Vocabulary;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.utils.FormularWorker;

public class HandleFormularsComposite extends Composite {
	// private static final String PATTERN_SIGNINPUT = "[!&|=()\\s]";
	private static final String PATTERN_SIGNINPUT = "[!&\\|=\\(\\)\\s]";

	/**
	 * Vocabulary which contains the formulars.
	 */
	private Vocabulary vocabulary;

	/**
	 * Textfield to type a formular
	 */
	private Text text;

	/**
	 * List with the formulars
	 */
	private List list;

	/**
	 * Label to show status messages to the user
	 */
	private Label labelStatus;

	/**
	 * Label to show position to user
	 */
	private Label labelPosition;

	/**
	 * Button to open the formular editor
	 */
	private Button openFormularEditorButton;

	/**
	 * Whole input String
	 */
	private String input = "";

	private String inputElement = "";

	/**
	 * Composite which holds input elements like text or position label
	 */
	private Composite inputComposite;

	/**
	 * List with formulars which are filterd by the current formular element
	 * given by the user.
	 */
	private java.util.List<String> filteredFormularNames;

	/**
	 * After every key event, the input is saved, to decide if a sign was added
	 * or removed.
	 */
	private String inputBefore = "";

	/**
	 * Flag to control if the list with formulars shell be updated, and the
	 * input formular shell be validated.
	 * 
	 * True if current input is a formular
	 * 
	 * false if current input is a proposition
	 * 
	 * Must be controlled from outside
	 */
	// private boolean handleInput = true;

	public HandleFormularsComposite(Composite parent, int style, Vocabulary vocabulary) {
		super(parent, style);
		this.vocabulary = vocabulary;
		setLayout(new GridLayout(1, false));
		setBackground(parent.getBackground());
		labelStatus = new Label(this, SWT.NONE);
		labelStatus.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, false, 1, 1));
		labelStatus.setText("");
		labelStatus.setBackground(getBackground());

		inputComposite = new Composite(this, SWT.NONE);
		getInputComposite().setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true, 1, 1));
		getInputComposite().setLayout(new GridLayout(2, false));
		getInputComposite().setBackground(getBackground());

		Label labelInput = new Label(getInputComposite(), SWT.NONE);
		labelInput.setText("Autocompletion:");
		labelInput.setBackground(getBackground());

		labelPosition = new Label(getInputComposite(), SWT.NONE);
		labelPosition.setText("Position: ");
		labelPosition.setBackground(getBackground());
		labelPosition.setLayoutData(new GridData(SWT.RIGHT, SWT.FILL, true, false, 1, 1));

		text = new Text(this, SWT.BORDER);
		getText().setLayoutData(new GridData(SWT.FILL, SWT.TOP, true, false, 1, 1));

		GridData listGridData = new GridData(SWT.FILL, SWT.TOP, true, true, 1, 1);
		listGridData.heightHint = 150;
		list = new List(this, SWT.BORDER | SWT.V_SCROLL | SWT.H_SCROLL);
		getList().setLayoutData(listGridData);

		openFormularEditorButton = new Button(this, SWT.PUSH);
		openFormularEditorButton.setText("Formular Editor");
		openFormularEditorButton.addSelectionListener(new OpenFormularSelectionListener());

		// global listeners
		parent.getShell().getDisplay().addFilter(SWT.KeyDown, new ArrowKeyListener());

		// element listeners
		getText().addKeyListener(new TextInputKeyListener());
		getText().addMouseListener(new UpdatePositionMouseListener());
		getList().addKeyListener(new ListKeyListener());
		getList().addMouseListener(new ListMouseListener());

		// inital values
		filteredFormularNames = vocabulary.getFormularNames();
		updateList(filteredFormularNames);

		// add property change listener to keep the list up to date if new
		// linked formular is added
		// formular added
		PropositionalLogicPlugin.getInstance().getLinkedFormularsVocabulary().addPropertyChangeListener(new UpdateFormularListListener());
		// new vocabulary via preference added
		PropositionalLogicPlugin.getInstance().addPropertyChangeListener(new UpdateFormularListListener());
	}

	/**
	 * Returns input as formular
	 * 
	 * @return
	 */
	public Formular getFormular() {
		Formular f = new Formular();
		f.setExpression(getFormularExpression());
		f.setElements(getFormularElements());

		return f;
	}

	/**
	 * Returns input as proposition.
	 * 
	 * @return
	 */
	public Proposition getProposition() {
		Proposition p = new Proposition();
		p.setExpression(getFormularExpression());

		return p;
	}

	/**
	 * Returns the formular expression as String
	 * 
	 * @return
	 */
	private String getFormularExpression() {
		return getText().getText();
	}

	/**
	 * Returns formular elements in a list
	 * 
	 * @return
	 */
	private java.util.List<String> getFormularElements() {
		return getFormularWorker().splitToElements(getText().getText());
	}

	/**
	 * Validates the formular and returns an {@link ValidationResult}-Instance,
	 * holding the results.
	 * 
	 * @return
	 */
	public ValidationResult isFormularValid() {
		Automat automat = new Automat();
		ValidationResult result = automat.getResult(getText().getText());

		if (!result.isValid()) {
			return result;
		} else {
			// check if every element is kwon by the vocabulary
			java.util.List<String> elements = getFormularWorker().splitToElements(getText().getText());
			for (String element : elements) {
				if (Pattern.matches(PluginConstants.REGEX_OPERAND, element)) {
					if (!getVocabulary().getFormularNames().contains(element)) {
						result = new ValidationResult(false, PluginConstants.STATUS_UNKOWN_FORMULAR + element, -1);
						break;
					}
				}
			}

			return result;
		}
	}

	/**
	 * Returns all formulars if sign equals " ", "!", "&", "|" "(", ")" all
	 * given formulars are returned, otherwise filterd formulars will be
	 * returned. For example, if <code>input</code> is "abc", only formulars
	 * with "abc" in the name are returned.
	 * 
	 * @param input
	 * @param formularNames
	 * @return
	 */
	private java.util.List<String> getFilteredFormulars(String input, java.util.List<String> formularNames) {
		if (" ".equals(input) || "|".equals(input) || "&".equals(input) || "!".equals(input) || "(".equals(input) || ")".equals(input)) {
			return formularNames;
		}

		java.util.List<String> filteredFormularNames = new ArrayList<String>();

		for (String formularName : formularNames) {
			if (formularName.contains(input)) {
				filteredFormularNames.add(formularName);
			}
		}
		Collections.sort(filteredFormularNames);
		return filteredFormularNames;
	}

	private Vocabulary getVocabulary() {
		return vocabulary;
	}

	private void setVocabulary(Vocabulary vocabulary) {
		this.vocabulary = vocabulary;
	}

	public Text getText() {
		return text;
	}

	public List getList() {
		return list;
	}

	/**
	 * Updates the autocompletion list. After calling this method only the given
	 * elements are shown in the list.
	 * 
	 * @param filteredFormuarNames
	 */
	private void updateList(java.util.List<String> filteredFormuarNames) {
		if (getList().isDisposed()) {
			return;
		}

		getList().removeAll();
		for (String formularName : filteredFormularNames) {
			getList().add(formularName);
		}
		getList().select(0);
		getList().update();
	}

	/**
	 * Returns the selected input element (formular or operator) in the
	 * textfield. An element is selected if the caret is inside or right after
	 * the element
	 * 
	 * @param inputString
	 * @param inputElements
	 * @param caretPosition
	 * @return
	 */
	private String getSelectedInputElement(String inputString, java.util.List<String> inputElements, int caretPosition) {
		String toHandle = inputString.substring(0, caretPosition);
		if ("".equals(toHandle)) {
			return "";
		} else if (' ' == toHandle.charAt(toHandle.length() - 1)) {
			return "";
		} else {
			java.util.List<String> handleInput = getFormularWorker().splitToElements(toHandle);
			return inputElements.get(handleInput.size() - 1);
		}
	}

	/**
	 * TODO
	 * 
	 * @param inputString
	 * @param toReplace
	 * @param inputElements
	 * @param caretPosition
	 * @return
	 */
	private java.util.List<String> replaceSelectedInputElement(String inputString, String toReplace, java.util.List<String> inputElements, int caretPosition) {
		String toHandle = inputString.substring(0, caretPosition);
		java.util.List<String> newElements = new ArrayList<String>();

		if ("".equals(toHandle)) {
			// formel am anfang der Liste einfügenregex
			newElements.add(toReplace);
			newElements.addAll(inputElements);
		} else if (' ' == toHandle.charAt(toHandle.length() - 1)) {
			// wenn letztes zeichen leerzeichen, dann zwischen zwei formeln
			// formel zwischen zwei formeln einfügen

			java.util.List<String> handleInput = getFormularWorker().splitToElements(toHandle);
			// String[] split = toHandle.split("\\s+");

			int index = handleInput.size() - 1;

			for (int i = 0; i < inputElements.size(); ++i) {
				if (i == index) {
					newElements.add(toReplace);
				}
				newElements.add(inputElements.get(i));

			}

		} else if (Pattern.matches("[!&\\|\\(\\)]", toHandle.charAt(toHandle.length() - 1) + "")) {
			// Formel nach operator oder klammer anhängen
			java.util.List<String> handleInput = getFormularWorker().splitToElements(toHandle);
			// size of this list is index in list of all inputelements
			int index = handleInput.size() - 1;
			for (int i = 0; i < inputElements.size(); ++i) {
				if (i == index) {
					newElements.add(toReplace);
				}
				newElements.add(inputElements.get(i));

			}
		} else {
			// zeichen durch formel ersetzen
			java.util.List<String> handleInput = getFormularWorker().splitToElements(toHandle);
			int index = handleInput.size() - 1;
			for (int i = 0; i < inputElements.size(); ++i) {
				if (i == index) {
					newElements.add(toReplace);
				} else {
					newElements.add(inputElements.get(i));
				}
			}
		}

		return newElements;
	}

	/**
	 * Prints inputelements to text in a formatted way
	 * 
	 * @param inputElements
	 */
	private void printInText(String input) {
		getText().setText("");
		getText().append(input);
		setPosition(getText().getCaretPosition());
	}

	/**
	 * Setts status in ui. If "valid" is true the background from status will be
	 * set to green, to inform user that everythind is fine. Otherwise it will
	 * be set to red.
	 * 
	 * @param status
	 * @param valid
	 */
	public void setStatus(String status, boolean valid) {
		labelStatus.setText(status);
		if (valid) {
			Device device = Display.getCurrent();
			Color background = new Color(device, 0, 205, 0);
			labelStatus.setBackground(background);

			Color foreground = Display.getCurrent().getSystemColor(SWT.COLOR_WHITE);
			labelStatus.setForeground(foreground);

		} else {
			Device device = Display.getCurrent();
			Color background = new Color(device, 238, 0, 0);
			labelStatus.setBackground(background);
			Color foreground = Display.getCurrent().getSystemColor(SWT.COLOR_WHITE);
			labelStatus.setForeground(foreground);
		}

		layout();

	}

	/**
	 * Shows position to user.
	 * 
	 * @param position
	 */
	private void setPosition(int position) {
		labelPosition.setText("Position: " + position);

		getInputComposite().layout();
	}

	/**
	 * Returns a formular worker instance to handle formulars.
	 * 
	 * @return
	 */
	private FormularWorker getFormularWorker() {
		return new FormularWorker();
	}

	private Composite getInputComposite() {
		return inputComposite;
	}


	public Button getOpenFormularEditorButton() {
		return openFormularEditorButton;
	}

	/**
	 * Analyses Input and filters formulars shown in List
	 * 
	 * @author philipp
	 * 
	 */
	private class TextInputKeyListener implements KeyListener {
		// String inputBefore;
		@Override
		public void keyPressed(KeyEvent e) {

		}

		@Override
		public void keyReleased(KeyEvent e) {
			// if (!handleInput) {
			// return;
			// }

			input = getText().getText();
			int caretPosition = getText().getCaretPosition();
			java.util.List<String> inputElements = getFormularWorker().splitToElements(input);
			inputElement = getSelectedInputElement(input, inputElements, caretPosition);
			// System.out.println("Input element: " + inputElement);

			// validateInput(input);

			// update postition label
			setPosition(caretPosition);

			// handle input
			if (e.keyCode == SWT.ARROW_LEFT || e.keyCode == SWT.ARROW_RIGHT || e.keyCode == SWT.END) {
				// brows trough formulars
				filteredFormularNames = getFilteredFormulars(inputElement, getVocabulary().getFormularNames());
				updateList(filteredFormularNames);
				return;
			} else if (Pattern.matches(PATTERN_SIGNINPUT, e.character + "") && e.keyCode != SWT.CR) {
				// Operand sign added
				/*
				 * CR matches also the pattern?! thats why e.keyCode != SWT.CR
				 * is checked
				 */
				filteredFormularNames = getVocabulary().getFormularNames();
				inputBefore = input;
				updateList(filteredFormularNames);
				return;
			} else if (Pattern.matches(PluginConstants.REGEX_OPERAND_SIGN, e.character + "")) {
				// sign added
				if ("".equals(inputBefore)) {
					// first call
					filteredFormularNames = getFilteredFormulars(inputElement, getVocabulary().getFormularNames());
				} else {
					filteredFormularNames = getFilteredFormulars(inputElement, filteredFormularNames);
				}
				inputBefore = input;
				updateList(filteredFormularNames);
				return;
			} else if (inputBefore.length() > input.length()) {
				// sign removed
				filteredFormularNames = getFilteredFormulars(inputElement, getVocabulary().getFormularNames());
				inputBefore = input;
				updateList(filteredFormularNames);
				return;
			} else if (e.keyCode == SWT.CR) {
				// enter pressed
				applySelectedFormular();
				inputBefore = input;
				filteredFormularNames = getVocabulary().getFormularNames();
				updateList(filteredFormularNames);
				return;
			} else {
				// default
				inputBefore = input;
				filteredFormularNames = getFilteredFormulars(inputElement, filteredFormularNames);
				updateList(filteredFormularNames);
				return;
			}

		}
	}

	/**
	 * Applies the selected formular from list to the correct position in the
	 * textfield.
	 */
	private void applySelectedFormular() {
		String input = getText().getText();
		int caretPosition = getText().getCaretPosition();
		java.util.List<String> inputElements = getFormularWorker().splitToElements(input);
		// System.out.println("List enter pressed");
		String[] selection = getList().getSelection();
		String toReplace = selection[0];
		if (selection.length > 0) {
			java.util.List<String> newElements = replaceSelectedInputElement(input, toReplace, inputElements, caretPosition);
			// System.out.println("newElements: " + newElements);
			String formatText = getFormularWorker().buildExpression(newElements);
			printInText(formatText);
			// check if new input is valid
			// validateInput()
		}

	}

	private class ListKeyListener implements KeyListener {

		@Override
		public void keyPressed(KeyEvent e) {
			// NOTHING TO DO

		}

		@Override
		public void keyReleased(KeyEvent e) {
			// String selectedElement = getSelectedInputElement(input,
			// inputElements, caretPosition);

			if (e.keyCode == SWT.CR) {
				applySelectedFormular();

				inputBefore = getText().getText();
				filteredFormularNames = getVocabulary().getFormularNames();
				updateList(filteredFormularNames);
				getText().setFocus();
			}

		}
	}

	private class ListMouseListener implements MouseListener {

		@Override
		public void mouseDoubleClick(MouseEvent e) {
			applySelectedFormular();

			inputBefore = getText().getText();
			filteredFormularNames = getVocabulary().getFormularNames();
			updateList(filteredFormularNames);
		}

		@Override
		public void mouseDown(MouseEvent e) {
			// NOTHING TO DO

		}

		@Override
		public void mouseUp(MouseEvent e) {
			// NOTHING TO DO

		}

	}

	private class UpdateFormularListListener implements PropertyChangeListener {

		@Override
		public void propertyChange(PropertyChangeEvent event) {
			if (PluginConstants.PC_VOCABULARY_CHANGED.equals(event.getPropertyName())) {
				// set new vocabulary with before added formular
				setVocabulary(PropositionalLogicPlugin.getInstance().getCompleteVocabulary());

				// from all formular, to filter also the new added formular
				filteredFormularNames = getVocabulary().getFormularNames();

				updateList(filteredFormularNames);
			}
		}

	}

	/**
	 * Listener to update position label if textfield gets the focus. Without
	 * this Listener the position is only set in label if input in textfield
	 * changes.
	 * 
	 * @author philipp
	 * 
	 */
	private class UpdatePositionMouseListener implements MouseListener {

		@Override
		public void mouseDoubleClick(MouseEvent e) {
			// NOTHING TO DO

		}

		@Override
		public void mouseDown(MouseEvent e) {
			// NOTHING TO DO

		}

		@Override
		public void mouseUp(MouseEvent e) {
			int caretPosition = getText().getCaretPosition();
			setPosition(caretPosition);
		}

	}

	/**
	 * If arrow key pressed, this global listener will select the list which
	 * will handle the event
	 * 
	 * @author philipp
	 * 
	 */
	private class ArrowKeyListener implements Listener {

		@Override
		public void handleEvent(Event event) {
			if (event.keyCode == SWT.ARROW_DOWN || event.keyCode == SWT.ARROW_UP) {
				if (!isDisposed()) {
					if (!getList().isFocusControl()) {
						getList().setFocus();
					}
				}
			}

		}

	}

	/**
	 * Selection Listener to pen formular explorer
	 */
	private class OpenFormularSelectionListener implements SelectionListener {

		@Override
		public void widgetSelected(SelectionEvent e) {
			FormularEditor editor = new FormularEditor(Display.getCurrent().getActiveShell());
			editor.open();

		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {
			// NOTHING TO DO

		}

	}
}
