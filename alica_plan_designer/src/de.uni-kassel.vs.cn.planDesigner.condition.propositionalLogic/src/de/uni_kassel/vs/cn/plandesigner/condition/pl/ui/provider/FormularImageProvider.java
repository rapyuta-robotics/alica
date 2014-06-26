package de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.provider;

import org.eclipse.jface.resource.ImageRegistry;
import org.eclipse.jface.viewers.TreeNode;
import org.eclipse.swt.graphics.Image;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.Activator;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Formular;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Proposition;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.IImageProvider;

/**
 * Imageprovider which returns images for formulars and propositions.
 * 
 * @author philipp
 * 
 */
public class FormularImageProvider implements IImageProvider {

	@Override
	public Image getImage(Object element) {
		Image image = null;
		ImageRegistry registry = Activator.getDefault().getImageRegistry();

		if (element instanceof TreeNode) {
			Object value = ((TreeNode) element).getValue();
			if (value instanceof Proposition) {
				image = registry.get(PluginConstants.ICON_PROPOSITION);
			} else if (value instanceof Formular) {
				image = registry.get(PluginConstants.ICON_FORMULAR);
			}else{
				image = registry.get(PluginConstants.ICON_FAILURE);
			}
		}

		return image;
	}

}
