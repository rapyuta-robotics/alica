package de.uni_kassel.vs.cn.plandesigner.condition.pl.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.PropositionalLogicPlugin;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Vocabulary;

/**
 * Class for operating with Strings which represents a propositional logic
 * formular.
 * 
 * @author philipp
 * 
 */
public class FormularWorker {

	/**
	 * Splitts a formular to its elements and returns a list with the elements.
	 * 
	 * @param input
	 * @return
	 */
	public List<String> splitToElements(String expression) {
		List<String> elements = new ArrayList<String>();
		String operand = "";
		for (Character c : expression.toCharArray()) {
			String sign = c + "";
			// if(Pattern.matches(PluginConstants.REGEX_OPERAND, sign)){
			//
			// }
			// else
			if (Pattern.matches(PluginConstants.REGEX_OPERATOR, sign)) {
				if (!"".equals(operand)) {
					elements.add(operand);
				}
				elements.add(sign);
				operand = "";
			} else if (Pattern.matches(PluginConstants.REGEX_WHITESPACE, sign)) {
				if (!"".equals(operand)) {
					elements.add(operand);
				}
				operand = "";

			} else {
				operand = operand + sign;
			}
		}

		elements.add(operand);
		return elements;
	}

	/**
	 * Returns true, if all elements in formular are resolved. An element is
	 * resolved if it is not a linked formular.
	 * 
	 * Method handles unkown operands like propositions.
	 * 
	 * @param formularElements
	 * @return
	 */
	public boolean isFormularResolved(List<String> formularElements) {
		Vocabulary linkedVocabulary = PropositionalLogicPlugin.getInstance().getLinkedFormularsVocabulary();

		for (String element : formularElements) {
			if (Pattern.matches(PluginConstants.REGEX_OPERAND, element)) {
				if (linkedVocabulary.contains(element)) {
					return false;
				}
			}
		}

		return true;
	}

	public String buildExpression(List<String> elements) {
		StringBuilder formattedText = new StringBuilder();
		for (int i = 0; i < elements.size(); ++i) {
			formattedText.append(elements.get(i));

			if (!"!".equals(elements.get(i)) && !"(".equals(elements.get(i))) {
				// if ((i + 1) < inputElements.size() && !inputElements.get(i +
				// 1).equals(PluginConstants.SIGN_CLOSED_BRACKET) || (i <
				// inputElements.size() - 1)) {
				// no whitespace if next sign is a closing bracket
				formattedText.append(" ");
				// }

			}
		}
		return formattedText.toString();
	}

}
