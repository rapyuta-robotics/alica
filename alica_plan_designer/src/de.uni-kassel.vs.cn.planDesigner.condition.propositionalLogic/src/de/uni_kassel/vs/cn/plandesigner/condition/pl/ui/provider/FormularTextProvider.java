package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider;

import org.eclipse.jface.viewers.TreeNode;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Formular;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.ITextProvider;

public class FormularTextProvider implements ITextProvider {

	@Override
	public String getText(Object element) {
		String result = "";
		if (element instanceof TreeNode) {
			Formular f = (Formular) ((TreeNode) element).getValue();
			result = f.getName() + ": " + f.getExpression();
		}

		return result;
	}

}
