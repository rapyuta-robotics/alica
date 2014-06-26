package de.uni_kassel.vs.cn.plandesigner.condition.pl;

import java.io.File;
import java.net.URL;

import org.eclipse.core.runtime.FileLocator;

import de.uni_kassel.vs.cn.planDesigner.ui.properties.PMLPropertySection;
import de.uni_kassel.vs.cn.plandesigner.condition.core.IConditionPlugin;
import de.uni_kassel.vs.cn.plandesigner.condition.core.templatebuilding.Template;
import de.uni_kassel.vs.cn.plandesigner.condition.core.templatebuilding.TemplateEngine;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.ui.PLPluginPropertySection;

public class PLPlugin implements IConditionPlugin {
	private static final String TEMPLATE_FILE = "/codegeneration/PluginTemplatePLP.xpt";

	@Override
	public String getName() {
		return "PropositionalLogicPlugin";
	}

	@Override
	public PMLPropertySection getUi() {
		return new PLPluginPropertySection();
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
			return null;
		}
	}

}
