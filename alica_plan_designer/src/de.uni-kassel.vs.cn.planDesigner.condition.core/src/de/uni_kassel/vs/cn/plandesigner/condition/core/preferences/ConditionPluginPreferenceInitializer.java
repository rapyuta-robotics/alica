package de.uni_kassel.vs.cn.plandesigner.condition.core.preferences;

import java.io.File;

import org.eclipse.core.runtime.preferences.AbstractPreferenceInitializer;
import org.eclipse.core.runtime.preferences.IEclipsePreferences;
import org.eclipse.core.runtime.preferences.InstanceScope;
import org.eclipse.jface.preference.IPreferenceStore;

import de.uni_kassel.vs.cn.plandesigner.condition.core.Activator;
import de.uni_kassel.vs.cn.plandesigner.condition.core.ConditionPluginLoader;

/**
 * Initializer for the preferences of this bundle.
 * 
 * preferences needed by system and not editable by user are setted everytime in
 * the Activator. (this class only sets default if no value exists) *
 * 
 * @author philipp
 * 
 */
public class ConditionPluginPreferenceInitializer extends AbstractPreferenceInitializer {

	public ConditionPluginPreferenceInitializer() {
		super();
	}

	@Override
	public void initializeDefaultPreferences() {
		IPreferenceStore preferenceStore = Activator.getDefault().getPreferenceStore();

		String rootPath;
		String es_root = System.getenv("ES_ROOT");
		if (es_root != null && !es_root.isEmpty()) {
			rootPath = es_root;
		} else {
			rootPath = System.getenv("HOME") + File.separator + "impera" + File.separator; 
		}

		/*
		 * Default folder for the Condition-Plugins.
		 * 
		 * Editable by user
		 */
		preferenceStore.setDefault(PreferenceConstants.PREF_CONDITION_PLUGIN_PATH, rootPath + "cn-alica-ros-pkg/Planmodeller/src/conditionPlugins");
		
		preferenceStore.setDefault(PreferenceConstants.PREF_TEMPLATE_INTERFACE_PATH, rootPath + "cn-alica-ros-pkg/Planmodeller/src/conditionPlugins/templates/TemplateInterface.xpt");
		
		preferenceStore.setDefault(PreferenceConstants.PREF_PLUGIN_TEMPLATE_PATH, rootPath + "cn-alica-ros-pkg/Planmodeller/src/conditionPlugins/templates/PluginTemplate.xpt");
				

		/*
		 * Default condition-plugin which starts if a condition is not modelled
		 * by any plugin.
		 * 
		 * Editable by user
		 */
		preferenceStore.setDefault(PreferenceConstants.PREF_DEFAULT_CONDITION_PLUGIN, "DefaultPlugin");

	
		preferenceStore.setDefault(PreferenceConstants.PREF_ENCODING, "ISO-8859-1");

	}

	private void removeAll() {
		IEclipsePreferences node = InstanceScope.INSTANCE.getNode(Activator.PLUGIN_ID);
		node.remove(PreferenceConstants.PREF_CONDITION_PLUGIN_PATH);
		node.remove(PreferenceConstants.PREF_DEFAULT_CONDITION_PLUGIN);

	}

}
