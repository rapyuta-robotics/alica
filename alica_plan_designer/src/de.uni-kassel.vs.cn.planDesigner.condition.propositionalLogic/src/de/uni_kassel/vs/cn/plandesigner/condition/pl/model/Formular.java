package de.uni_kassel.vs.cn.plandesigner.condition.pl.model;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.automat.Automat;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.automat.ValidationResult;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.utils.FormularWorker;

public class Formular {

	/**
	 * Name of formular
	 */
	protected String name;
	/**
	 * Expression of formular
	 */
	protected String expression;

	/**
	 * All elements an expression consists of
	 */
	protected List<String> elements;

	protected List<Formular> childs;

	// /**
	// * True if formular is a proposition, false if it is a linked formular.
	// */
	// private boolean isProposition;

	// public Formular(boolean isProposition){
	// this.isProposition = isProposition;
	// }

	private void addToChilds(Formular child) {

		getChilds().add(child);
	}

	public List<Formular> getChilds() {
		if (childs == null) {
			childs = new ArrayList<Formular>();
		}
		return this.childs;
	}

	/**
	 * Returns name of formular.
	 * 
	 * @return
	 */
	public String getName() {
		return name;
	}

	/**
	 * Sets attribute name to value.
	 * 
	 * @param name
	 */
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * Returns the formulars expression.
	 * 
	 * @return
	 */
	public String getExpression() {
		return expression;
	}

	/**
	 * Sets expression of formular. Also modifies attribute elements.
	 * 
	 * @param expression
	 */
	public void setExpression(String expression) {
		if (expression != null && !expression.equals(this.expression)) {
			this.expression = expression;
			FormularWorker worker = new FormularWorker();
			List<String> elements = worker.splitToElements(expression);
			setElements(elements);
		}
	}

	/**
	 * Returns elements the expression of formular consists of.
	 * 
	 * @return
	 */
	public List<String> getElements() {
		return elements;

	}

	/**
	 * Set elements of formular. Also modifies attribute expression.
	 * 
	 * @param elements
	 */
	public void setElements(List<String> elements) {
		if (elements != null && !elements.equals(this.elements)) {
			this.elements = elements;
			FormularWorker worker = new FormularWorker();
			String expression = worker.buildExpression(elements);
			setExpression(expression);
		}

	}

	/**
	 * Returns true if formular is resolved. A formular is resolved if no
	 * operand is an linked formular.
	 * 
	 * @return
	 */
	public boolean isResolved() {
		return new FormularWorker().isFormularResolved(getElements());
	}

	public ValidationResult isValid() {
		Automat automat = new Automat();
		ValidationResult result = automat.getResult(expression);
		
		if (!result.isValid()) {
			return result;
		} else {
			// check if every element is kwon by the vocabulary
			for (String element : getElements()) {
				if (Pattern.matches(PluginConstants.REGEX_OPERAND, element)) {
					if (!PropositionalLogicPlugin.getInstance().getCompleteVocabulary().getFormularNames().contains(element)) {
						result = new ValidationResult(false, PluginConstants.STATUS_UNKOWN_FORMULAR + element, -1);
						break;
					}
				}
			}
			
			return result;
		}
		
	}

	// public boolean isProposition() {
	// return isProposition;
	// }
	//
	// public void setProposition(boolean isProposition) {
	// this.isProposition = isProposition;
	// }

	/**
	 * Returns resolved formular. A formular is resolved if no operand is an
	 * linked formular.
	 * 
	 * @return
	 */
	public String getResolvedExpression() {
		FormularWorker worker = new FormularWorker();
		String expression = worker.buildExpression(getResolvedElements());
		return expression;
	}

	/**
	 * Returns elements of the resolved formular.
	 * 
	 * @return
	 */
	public List<String> getResolvedElements() {
		List<String> resolvedElements = new ArrayList<String>();
		resolvedElements.addAll(getElements());
		List<String> temp = new ArrayList<String>();

		Vocabulary linkedFormularsVocabulary = PropositionalLogicPlugin.getInstance().getLinkedFormularsVocabulary();

		FormularWorker worker = new FormularWorker();
		while (!worker.isFormularResolved(resolvedElements)) {
			for (String element : resolvedElements) {
				if (Pattern.matches(PluginConstants.REGEX_OPERAND, element)) {
					if (linkedFormularsVocabulary.contains(element)) {
						Formular f = linkedFormularsVocabulary.getFormular(element);
						temp.add(PluginConstants.SIGN_OPEN_BRACKET);
						temp.addAll(f.getElements());
						temp.add(PluginConstants.SIGN_CLOSED_BRACKET);
						continue;
					}
				}
				temp.add(element);
			}

			resolvedElements = temp;
			temp = new ArrayList<String>();
		}

		return resolvedElements;
	}

	/**
	 * Returns all resolved operands off the formular.
	 * 
	 * @return
	 */
	public List<String> getOperands() {
		List<String> operands = new ArrayList<String>();
		for (String element : getResolvedElements()) {
			if (Pattern.matches(PluginConstants.REGEX_OPERAND, element)) {
				if (!operands.contains(element)) {
					operands.add(element);
				}
			}
		}

		return operands;
	}

	/**
	 * Adds childs to formular. Only call this method if the vocabularies are
	 * created.
	 */
	public void addChilds() {
		addChildsToFormular(this);
	}

	private void addChildsToFormular(Formular f) {
		for (String element : f.getElements()) {
			Formular child = PropositionalLogicPlugin.getInstance().getCompleteVocabulary().getFormular(element);
			if (child == null) {
				continue;
			} else if (child instanceof Formular) {
				if (!f.getChilds().contains(child)) {
					f.addToChilds(child);
					addChildsToFormular(child);
				}
			} else if (child instanceof Proposition) {
				if (!f.getChilds().contains(child)) {
					f.addToChilds(child);
				}
			}
		}
	}
}
