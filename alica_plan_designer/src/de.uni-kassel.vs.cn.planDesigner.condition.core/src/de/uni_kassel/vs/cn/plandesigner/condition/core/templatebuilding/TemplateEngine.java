package de.uni_kassel.vs.cn.plandesigner.condition.core.templatebuilding;

import java.nio.charset.Charset;
import java.util.List;

/**
 * The Template Engine builds the aspect based template for the codegenerator.
 * 
 * Inserts all aspects from the different plugin templates to the main plugin
 * template in the codegenerator bundle.
 * 
 * @author philipp
 * 
 */
public class TemplateEngine {
	/**
	 * Encoding of the template file, must be an encoding which contains « and »
	 */
	private Charset encoding;

	public TemplateEngine() {
		encoding = Charset.forName("ISO-8859-1");
	}

	/**
	 * Creates the main plugin template for the codegenerator by the different
	 * plugin templates.
	 * 
	 * First method checks if all plugins implement all aspects defined in the
	 * main plugin template, then the aspects are inserted to the main plugin
	 * template.
	 * 
	 * @param templateInterface
	 * @param pluginTemplates
	 * @return
	 */
	public String createPluginTemplate(Template templateInterface, List<Template> pluginTemplates, Template toGenerate) {
		// parse aspects
		
		//remove aspects from earlier generation of the template
		toGenerate.getAspects().clear();

		String error = checkAspects(templateInterface, pluginTemplates);
		if (error != null) {
			return error;
		}

		// add the plugin aspects
		for (Aspect mainAspect : templateInterface.getAspects()) {
			for (Template pluginTemplate : pluginTemplates) {
				Aspect pluginAspect = pluginTemplate.getAspect(mainAspect.getName());
							
				// plugin templates contain only one aspect code instance
				if (pluginAspect.getCodes().size() > 0) {
					mainAspect.addToCodes(pluginAspect.getCodes().get(0));
				}
			}
			
			toGenerate.getAspects().add(mainAspect);
		}

		return null;
	}

	/**
	 * Checks if all aspects are provided by the template file
	 * 
	 * @param aspectNames
	 * @param pluginTemplate
	 * @return
	 */
	private String checkAspects(Template mainTemplate, List<Template> pluginTemplates) {
		String error = null;
		for (Aspect aspect : mainTemplate.getAspects()) {
			for (Template pluginTemplate : pluginTemplates) {
				if (!pluginTemplate.containsAspect(aspect)) {
					error = "Missing aspect: " + aspect.getName() + " in file: " + pluginTemplate.getName();
					return error;
				}
			}
		}

		return null;
	}
}
