package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui;

import org.eclipse.jface.action.IContributionItem;
import org.eclipse.ui.actions.CompoundContributionItem;

public class TestDynamicMenu extends CompoundContributionItem{

	@Override
	protected IContributionItem[] getContributionItems() {
		System.out.println("CALLED METHOD");
		return null;
	}

}
