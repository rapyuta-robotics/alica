package de.uni_kassel.vs.cn.plandesigner.condition.defaultPlugin;

import java.io.File;
import java.net.URL;

import javax.print.attribute.standard.MediaSize.Engineering;

import org.eclipse.core.runtime.FileLocator;

import de.uni_kassel.vs.cn.planDesigner.ui.properties.PMLPropertySection;
import de.uni_kassel.vs.cn.plandesigner.condition.core.IConditionPlugin;
import de.uni_kassel.vs.cn.plandesigner.condition.core.templatebuilding.Template;
import de.uni_kassel.vs.cn.plandesigner.condition.core.templatebuilding.TemplateEngine;

public class DefaultPlugin implements IConditionPlugin {
	private static final String TEMPLATE_FILE = "/codegeneration/PluginTemplateDefaultPlugin.xpt";

	@Override
	public String getName() {
		return "DefaultPlugin";
	}

	@Override
	public PMLPropertySection getUi() {
		return new DefaultPluginPropertySection();
	}

	@Override
	public Template getTemplateFile() {
		try {
			URL resource = FileLocator.toFileURL(getClass().getResource(TEMPLATE_FILE));
			File file;
			file = new File(resource.toURI());
			Template template = new Template(file);
			return template;
		} catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}
}