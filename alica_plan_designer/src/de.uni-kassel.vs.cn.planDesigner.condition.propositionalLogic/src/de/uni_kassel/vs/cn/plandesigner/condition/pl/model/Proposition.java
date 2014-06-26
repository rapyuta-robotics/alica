package de.uni_kassel.vs.cn.plandesigner.condition.pl.model;

import java.util.ArrayList;
import java.util.List;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.automat.ValidationResult;

public class Proposition extends Formular {

	@Override
	public void setExpression(String expression) {
		if (expression != null && !expression.equals(this.expression)) {
			this.expression = expression;
			List<String> elements = new ArrayList<String>();
			elements.add(expression);
			setElements(elements);
		}
	}
	
	@Override
	public void setElements(List<String> elements) {
		if(elements != null && !elements.equals(this.elements)){
			this.elements = elements;
			String expression = elements.get(0);
			setExpression(expression);
		}
	}
	
	@Override
	public ValidationResult isValid() {
		//proposition is always valid
		ValidationResult result = new ValidationResult(true, PluginConstants.STATUS_PROPOSITION_IS_VALID, -1);
		return result;
	}
}
