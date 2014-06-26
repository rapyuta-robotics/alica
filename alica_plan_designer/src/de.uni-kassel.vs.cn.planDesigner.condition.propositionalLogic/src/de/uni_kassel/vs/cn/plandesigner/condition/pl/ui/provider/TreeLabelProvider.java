package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider;

import java.awt.image.ImageProducer;

import org.eclipse.jface.resource.ImageRegistry;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.viewers.TreeNode;
import org.eclipse.swt.graphics.Image;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.Activator;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Formular;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Proposition;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.IImageProvider;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.ITextProvider;

/**
 * Label provider to show informations in a tree viewer. Needs an
 * {@link ITextProvider}-Instance and {@link IImageProvider}-Instance to show
 * the information
 * 
 * @author philipp
 * 
 */
public class TreeLabelProvider extends LabelProvider {
	/**
	 * Provider for the text.
	 */
	private ITextProvider textProvider;
	/**
	 * Provider for the image
	 */
	private IImageProvider imagerProvider;

	/**
	 * 
	 * @param textProvider
	 *            Concrete provider for the text.
	 * @param imagerProvider
	 *            Concrete provider for the image
	 */
	public TreeLabelProvider(ITextProvider textProvider, IImageProvider imagerProvider) {
		this.textProvider = textProvider;
		this.imagerProvider = imagerProvider;

	}

	@Override
	public String getText(Object element) {
		return textProvider.getText(element);
	}

	@Override
	public Image getImage(Object element) {
		return imagerProvider.getImage(element);
	}
}
