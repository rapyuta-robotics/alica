package de.uni_kassel.vs.cn.plandesigner.condition.pl.model;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;

import org.eclipse.jface.viewers.TreeNode;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.MessageBox;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.automat.Automat;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.automat.ValidationResult;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.utils.FormularParser;

public class PropositionalLogicPlugin {
	public static final String PROPERTY_CHANGE_NEW_FORMULAR = "property_change_new_formular";
	public static final String PROPERTY_CHANGE_LINKED_FORMULAR = "property_change_linked_formular";

	/**
	 * Singleton instance
	 */
	private static PropositionalLogicPlugin instance;

	/**
	 * Vocabulary with propositions
	 */
	private Vocabulary propositionVocabulary;

	/**
	 * Vocabulary with linked formulars
	 */
	private Vocabulary linkedFormularsVocabulary;

	/**
	 * Property Change Support for notifing listeners
	 */
	private static PropertyChangeSupport propertyChangeSupport;

	public static PropositionalLogicPlugin getInstance() {
		if (instance == null) {
			instance = new PropositionalLogicPlugin();

			FormularParser parser = new FormularParser();

			Vocabulary vocabulary = parser.getLinkedFormularVocabulary();
			vocabulary.setPropertyChangeSupport(getPropertyChangeSupport());

			instance.setLinkedFormularsVocabulary(vocabulary);

			vocabulary = parser.getPropositionVocabulary();
			vocabulary.setPropertyChangeSupport(getPropertyChangeSupport());

			instance.setPropositionVocabulary(vocabulary);

			if (instance.getLinkedFormularsVocabulary().isEmpty() || instance.getPropositionVocabulary().isEmpty()) {
				MessageBox mb = new MessageBox(Display.getCurrent().getActiveShell());
				mb.setText(PluginConstants.HEADLINE_PROBLEM_VOCABULARY);
				mb.setMessage(PluginConstants.STATUS_PROBLEMS_WITH_VOCABULARIES);
				mb.open();
			}

			// set the formulars childs
			for (String name : getInstance().getLinkedFormularsVocabulary().getFormularNames()) {
				Formular f = getInstance().getLinkedFormularsVocabulary().getFormular(name);
				f.addChilds();
			}
		}

		return instance;
	}

	/**
	 * Returns Vocabulary with propositions and linked formulars. This
	 * vocabulary is created as a copy of the propositions and linked formulars
	 * vocabulary. You can't call the save()-method on this vocabulary.
	 * 
	 * @return
	 */
	public Vocabulary getCompleteVocabulary() {
		Vocabulary vocabulary = new Vocabulary();
		for (String s : getPropositionVocabulary().getFormularNames()) {
			vocabulary.addToFormulars(getPropositionVocabulary().getFormular(s));
		}

		for (String s : getLinkedFormularsVocabulary().getFormularNames()) {
			vocabulary.addToFormulars(getLinkedFormularsVocabulary().getFormular(s));
		}

		return vocabulary;
	}

	private PropositionalLogicPlugin() {
		// singleton
	}

	/**
	 * Validates all formulars in the linked formulars vocabulary. Map contains
	 * the formulars and corresponding error messages.
	 * 
	 * @return
	 */
	public Map<Formular, List<String>> validateFormulars() {
		Map<Formular, List<String>> errorMap = new HashMap<Formular, List<String>>();
		List<String> errors;

		Vocabulary v = getLinkedFormularsVocabulary();
		Automat automat = new Automat();
		for (String name : v.getFormularNames()) {
			Formular f = v.getFormular(name);

			errors = new ArrayList<String>();
			// check syntax
			ValidationResult result = automat.getResult(f.getExpression());
			boolean isValid = result.isValid();
			if (!isValid) {
				errors.add(result.getStatusMessage());
			}

			// check if every element is given by the vocabularies
			List<String> elements = f.getElements();
			for (String element : elements) {
				if (Pattern.matches(PluginConstants.REGEX_OPERAND, element)) {
					if (!getCompleteVocabulary().contains(element)) {
						errors.add(PluginConstants.STATUS_UNKOWN_FORMULAR + element);
					}
				}
			}

			if (!errors.isEmpty()) {
				errorMap.put(f, errors);
			}
		}

		return errorMap;
	}

	/**
	 * Creates formula tree from the complete vocabulary
	 * 
	 * @return
	 */
	public List<TreeNode> getFormulaTree() {
		List<TreeNode> formulaTree = new ArrayList<TreeNode>();

		for (String name : getInstance().getCompleteVocabulary().getFormularNames()) {
			Formular f = getInstance().getCompleteVocabulary().getFormular(name);
			// TreeNode newNode = new TreeNode(f.getName() + ": " +
			// f.getExpression());
			TreeNode newNode = new TreeNode(f);

			// List<TreeNode> nodeChilds = new ArrayList<TreeNode>();
			addChilds(newNode, f);

			formulaTree.add(newNode);
		}
		return formulaTree;
	}

	private void addChilds(TreeNode node, Formular f) {
		if (f.getChilds().size() > 0) {
			TreeNode[] childs = new TreeNode[f.getChilds().size()];
			for (int i = 0; i < f.getChilds().size(); ++i) {
				Formular child = f.getChilds().get(i);
				// childs[i] = new TreeNode(child.getName() + ": " +
				// child.getExpression());
				childs[i] = new TreeNode(child);
				if (child instanceof Formular) {
					addChilds(childs[i], child);
				}
			}

			node.setChildren(childs);
		}
	}

	public void addPropertyChangeListener(PropertyChangeListener listener) {
		getPropertyChangeSupport().addPropertyChangeListener(listener);
	}

	public void removePropertyChangeListener(PropertyChangeListener listener) {
		getPropertyChangeSupport().removePropertyChangeListener(listener);
	}

	private static PropertyChangeSupport getPropertyChangeSupport() {
		if (propertyChangeSupport == null) {
			propertyChangeSupport = new PropertyChangeSupport(instance);
		}

		return propertyChangeSupport;
	}

	public Vocabulary getPropositionVocabulary() {
		return this.propositionVocabulary;
	}

	public void setPropositionVocabulary(Vocabulary value) {
		Vocabulary oldValue = this.propositionVocabulary;
		this.propositionVocabulary = value;
		getPropertyChangeSupport().firePropertyChange(PluginConstants.PC_VOCABULARY_CHANGED, oldValue, value);

	}

	public void setLinkedFormularsVocabulary(Vocabulary value) {
		Vocabulary oldValue = this.linkedFormularsVocabulary;
		this.linkedFormularsVocabulary = value;
		getPropertyChangeSupport().firePropertyChange(PluginConstants.PC_VOCABULARY_CHANGED, oldValue, value);
	}

	public Vocabulary getLinkedFormularsVocabulary() {
		return this.linkedFormularsVocabulary;
	}
}
