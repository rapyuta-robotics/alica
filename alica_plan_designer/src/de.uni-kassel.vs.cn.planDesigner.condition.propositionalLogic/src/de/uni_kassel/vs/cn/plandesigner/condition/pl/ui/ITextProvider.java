package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider.TreeLabelProvider;

/**
 * Interface for concrete text providers for the {@link TreeLabelProvider}
 * @author philipp
 *
 */
public interface ITextProvider  {

	public String getText(Object o);
}
