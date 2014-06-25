package de.uni_kassel.vs.cn.plandesigner.condition.core;

import de.uni_kassel.vs.cn.planDesigner.ui.properties.PMLPropertySection;
import de.uni_kassel.vs.cn.plandesigner.condition.core.templatebuilding.Template;

/**
 * Interface which must be implemented by all bundles with the condition plugin
 * extension.
 * 
 * @author philipp
 * 
 */
public interface IConditionPlugin {

	/**
	 * Returns name of the condition plugin
	 * @return
	 */
	public String getName();

	/**
	 * Returns UI of the condition plugin
	 * @return
	 */
	public PMLPropertySection getUi();

	/**
	 * Returns the template file for the codegeneration
	 * @return
	 */
	public Template getTemplateFile();

}
