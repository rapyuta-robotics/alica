package de.uni_kassel.vs.cn.plandesigner.condition.pl.preferences;

import java.io.File;

import org.eclipse.core.runtime.preferences.AbstractPreferenceInitializer;
import org.eclipse.core.runtime.preferences.IEclipsePreferences;
import org.eclipse.core.runtime.preferences.InstanceScope;
import org.eclipse.jface.preference.IPreferenceStore;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.Activator;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;

public class PLPluginPreferenceInitializer extends AbstractPreferenceInitializer{
	public PLPluginPreferenceInitializer(){
		super();
	}
	
	@Override
	public void initializeDefaultPreferences() {
		IPreferenceStore preferenceStore = Activator.getDefault().getPreferenceStore();
		
		//TODO ES_ROOT Variable irgendwie nicht verfügbar, erstmal mit $HOME
		String rootPath;
		String es_root = System.getenv("ES_ROOT");
		if (es_root != null && !es_root.isEmpty()){
			rootPath =  es_root;
		} else {
			rootPath = System.getenv("HOME") + File.separator + "impera" + File.separator;
		}
		preferenceStore.setDefault(PluginConstants.PREFERENCE_LINKED_FORMULAR_VOCABULARY_PATH, rootPath + "/cn-alica-ros-pkg/Planmodeller/src/conditionPlugins/vocabularies/linked_formulars.txt");
		preferenceStore.setDefault(PluginConstants.PREFERENCE_PROPOSITION_VOCABULARY_PATH, rootPath + "/cn-alica-ros-pkg/Planmodeller/src/conditionPlugins/vocabularies/propositions.txt");
		
		//removeAll();
		
		
		//System.out.println(preferenceStore);
	}
	
	private void removeAll(){
		System.out.println("PLPuginPreferenceInitializer.removeAll()");
		IEclipsePreferences node = InstanceScope.INSTANCE.getNode(Activator.PLUGIN_ID);
		node.remove(PluginConstants.PREFERENCE_LINKED_FORMULAR_VOCABULARY_PATH);
		node.remove(PluginConstants.PREFERENCE_PROPOSITION_VOCABULARY_PATH);
		
		try{
			node.flush();
		}catch(Exception e){
			e.printStackTrace();
		}
	}

}
