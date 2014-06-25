package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider;

import org.eclipse.jface.viewers.TreeNode;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.ITextProvider;

public class StringTextProvider implements ITextProvider {

	@Override
	public String getText(Object element) {
		String result = (String) ((TreeNode) element).getValue();

		return result;
	}

}
