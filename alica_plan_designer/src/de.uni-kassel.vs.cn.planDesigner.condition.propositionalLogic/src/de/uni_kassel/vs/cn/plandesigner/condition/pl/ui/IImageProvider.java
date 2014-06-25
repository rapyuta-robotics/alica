package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui;

import org.eclipse.swt.graphics.Image;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider.TreeLabelProvider;

/**
 * Interface for concrete image providers for the {@link TreeLabelProvider}
 * @author philipp
 *
 */
public interface IImageProvider {
	public Image getImage(Object o);

}
