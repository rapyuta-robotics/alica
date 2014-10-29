package de.uni_kassel.vs.cn.plandesigner.condition.core;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.management.RuntimeErrorException;

import org.eclipse.core.internal.runtime.InternalPlatform;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IConfigurationElement;
import org.eclipse.core.runtime.IExtensionRegistry;
import org.eclipse.core.runtime.Platform;
import org.eclipse.jface.action.IToolBarManager;
import org.eclipse.jface.preference.IPreferenceStore;
import org.eclipse.swt.widgets.MessageBox;
import org.eclipse.ui.IWorkbench;
import org.eclipse.ui.IWorkbenchWindow;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.internal.WorkbenchWindow;
import org.osgi.framework.Bundle;
import org.osgi.framework.BundleContext;
import org.osgi.framework.BundleException;

import de.uni_kassel.vs.cn.plandesigner.condition.core.preferences.PreferenceConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.core.templatebuilding.Template;
import de.uni_kassel.vs.cn.plandesigner.condition.core.templatebuilding.TemplateEngine;

/**
 * Class for loading condition plugins at runtime.
 * 
 * @author philipp
 * 
 */
public class ConditionPluginLoader {
	// Constants for property change support
	public static final String PROP_INSTALLED_BUNDLES = "prop_installed_bundles";
	public static final String PROP_UNINSTALLED_BUNDLES = "prop_uninstalled_bundles";
	public static final String PROP_REFRESHED_PLUGINS = "prop_refreshed_plugins";
	public static final String PROP_SET_PLUGINPATH = "prop_set_pluginpath";

	/**
	 * Property Changes Support for notifing different listeners for changes in
	 * the {@link ConditionPluginLoader}-Instance.
	 */
	private PropertyChangeSupport propertyChangeSupport;

	/**
	 * List of installed bundles which are condition plugins
	 */
	private List<Bundle> installedBundles;

	/**
	 * All condition plugins
	 */
	private Map<String, IConditionPlugin> plugins;

	/**
	 * The Path to the PluginFolder
	 */
	private String pluginPath;

	/**
	 * The current used condition plugins from the user
	 */
	private IConditionPlugin activeConditionPlugin;

	/**
	 * The preference store to access config values
	 */
	private IPreferenceStore store;

	/**
	 * Instance of this object
	 */
	private static ConditionPluginLoader instance;

	public static ConditionPluginLoader getInstance() {
		if (instance == null) {
			instance = new ConditionPluginLoader();
		}

		return instance;
	}

	/**
	 * Initialization code of this class
	 */
	public void init() {
		/*
		 * in extra thread because some of the conditionplugin adds ui elements
		 * to the PlanDesigner, in this case the PlanDesigner Workbench must be
		 * created. This thread waits until the workbench is created and then
		 * installs the condition plugins.
		 */

		new Thread() {
			public void run() {
				while (!PlatformUI.isWorkbenchRunning()) {
					// busy waiting
					try {
						Thread.sleep(500);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}

				PlatformUI.getWorkbench().getDisplay().syncExec(new Runnable() {

					@Override
					public void run() {

						propertyChangeSupport = new PropertyChangeSupport(this);
						final String pluginPath = getPluginPath();

						// install bundles, then extract plugins
						installedBundles = installBundles(pluginPath);

						if (installedBundles == null || installedBundles.isEmpty()) {
							showMessageBox("Problems with condition plugins", "Condition plugin folder is empty or does not contain any bundle");
						}

						plugins = getConditionPlugins();
						copyGenerationFiles();
					}

				});

			

			}

		}.start();

		store = Activator.getDefault().getPreferenceStore();
	}

	/**
	 * Method for finish the condition plugins. Uninstalls all bundles.
	 */
	public void finish() {
		// uninstall condition plugins
		for (Bundle bundle : getInstalledBundles()) {
			try {
				bundle.uninstall();
			} catch (BundleException e) {
				e.printStackTrace();
			}
		}
	}

	// only available via getInstance()
	private ConditionPluginLoader() {

	}

	/**
	 * Shows MessageBox with given text and message to user.
	 * 
	 * @param text
	 * @param message
	 */
	private void showMessageBox(String text, String message) {
		 MessageBox mb = new
		 MessageBox(PlatformUI.getWorkbench().getDisplay().getActiveShell());
		 mb.setText(text);
		 mb.setMessage(message);
		 mb.open();
	}

	/**
	 * Returns a bundle by its symbolic name
	 * 
	 * @param symbolicName
	 * @return
	 */
	private Bundle getBundle(String symbolicName) {
		BundleContext bundleContext = InternalPlatform.getDefault().getBundleContext();

		Bundle[] bundles = bundleContext.getBundles();
		Bundle bundle = null;

		for (Bundle b : bundles) {
			if (b.getSymbolicName().equals(symbolicName)) {
				bundle = b;
				break;
			}
		}

		return bundle;
	}

	/**
	 * This Method creates the PluginTemplate File in the codegeneration bundle.
	 * Therefore it copies the aspects given in the template files from the
	 * different condition plugins to one PluginTemplate in the codegeneration
	 * bundle. You can set the PluginTemplate File in the codegeneration
	 * project.
	 * 
	 * The method is able to copy the generation files to the codegeneration
	 * project (if Plan Designer was started via eclipse) or to the
	 * codegeneration bundle (if Plan Designer was started standalone)
	 * 
	 * @throws Exception
	 * 
	 */
	private void copyGenerationFiles() {
		if (getConditionPlugins().isEmpty()) {
			return;
		}

		Bundle codeGenBundle = getBundle("de.uni-kassel.vs.cn.planDesigner.codegeneration");

		if (codeGenBundle == null) {
			System.out.println("Couldn't copy codegenfiles, because de.uni-kassel.vs.cn.planDesigner.codegeneration not found");
			return;
		}

		// get the templates from the plugin
		List<Template> pluginTemplates = new ArrayList<Template>();
		for (String name : getConditionPluginNames()) {
			IConditionPlugin plugin = getConditionPlugin(name);
			pluginTemplates.add(plugin.getTemplateFile());
		}

		IPreferenceStore preferenceStore = Activator.getDefault().getPreferenceStore();

		String pathTemplateInterface = preferenceStore.getString(PreferenceConstants.PREF_TEMPLATE_INTERFACE_PATH);
		String pathPluginTemplate = preferenceStore.getString(PreferenceConstants.PREF_PLUGIN_TEMPLATE_PATH);

		Template templateInterface = new Template(new File(pathTemplateInterface));
		Template pluginTemplate = new Template(new File(pathPluginTemplate));

		TemplateEngine engine = new TemplateEngine();

		// sideeffect on pluginTemplate
		engine.createPluginTemplate(templateInterface, pluginTemplates, pluginTemplate);

		pluginTemplate.persist();

	}

	/**
	 * Performs new lookup for condition plugins in the folder given in the
	 * prefere.nces
	 */
	public void refreshConditionPlugins() {
		Map<String, IConditionPlugin> oldPlugins = plugins;

		String pluginPath = getPluginPath();
		if(getInstalledBundles().size() != 0)
		{
			uninstallConditionPlugins(getInstalledBundles());
		}
		installedBundles = installBundles(pluginPath);
		plugins = getConditionPlugins();

		// set the new version of the active conditionplugin
		if (getActiveConditionPlugin() != null) {
			setActiveConditionPlugin(plugins.get(getActiveConditionPlugin().getName()));
		}
		// recreate the generation files
		copyGenerationFiles();

//		firePropertyChange(PROP_REFRESHED_PLUGINS, oldPlugins, plugins);
	}

	/**
	 * Returns a Set with the names of the ConditionPlugins
	 * 
	 * @return
	 */
	public Set<String> getConditionPluginNames() {
		if (plugins != null) {
			return plugins.keySet();
		}

		return null;
	}

	/**
	 * Returns condition plugin with the given name.
	 * 
	 * @param name
	 * @return
	 */
	public IConditionPlugin getConditionPlugin(String name) {
		return plugins.get(name);
	}

	/**
	 * Returns the condition-plugins. Before calling this method, the bundles
	 * with conditin-plugins must be installed.
	 * 
	 * @return
	 */
	private Map<String, IConditionPlugin> getConditionPlugins() {
		Map<String, IConditionPlugin> conditionPlugins = new HashMap<String, IConditionPlugin>();

		IExtensionRegistry extensionRegistry = Platform.getExtensionRegistry();
		IConfigurationElement[] elements = extensionRegistry.getConfigurationElementsFor("de.uni_kassel.vs.cn.plandesigner.condition");

		for (IConfigurationElement element : elements) {
			try {
				Object o = element.createExecutableExtension("class");
				if (o instanceof IConditionPlugin) {
					IConditionPlugin plugin = (IConditionPlugin) o;
					conditionPlugins.put(plugin.getName(), plugin);
				}
			} catch (CoreException e) {
				e.printStackTrace();
			}
		}

		return conditionPlugins;
	}

	/**
	 * Install the bundles in the folder which holds the bundles with condition
	 * plugins.
	 * 
	 * @þaram pluginPath Folder which contains the condition plugins.
	 * 
	 * @return
	 */
	private List<Bundle> installBundles(String pluginPath) {
		// save old reference for prop. change
		List<Bundle> oldBundles = getInstalledBundles();

		if (pluginPath == null || "".equals(pluginPath)) {
			System.out.println("No pluginpath set.");
			return new ArrayList<Bundle>();
		}
		List<Bundle> installedBundles = new ArrayList<Bundle>();
		try {
			File pluginFolder = new File(pluginPath);

			// context to install the plugins
			BundleContext bundleContext = InternalPlatform.getDefault().getBundleContext();

			for (File pluginFile : pluginFolder.listFiles()) {
				if (pluginFile.getName().contains(".jar")) {
					String path = pluginFile.getAbsolutePath();
					Bundle bundle = bundleContext.installBundle("file://" + path);
					bundle.start();

					installedBundles.add(bundle);

				}
			}
		} catch (NullPointerException e) {
			// pluginfolder doesnt exist or is empty
			System.out.println("Pluginfolder don't exist or is empty");
			installedBundles = new ArrayList<Bundle>();

		} catch (BundleException e) {
			showMessageBox("Probles while installing condition plugin", e.getMessage());
			e.printStackTrace();
		}

//		firePropertyChange(PROP_INSTALLED_BUNDLES, installedBundles, oldBundles);

		return installedBundles;
	}

	/**
	 * Uninstall all installed bundles and removes the items from the parameter
	 * installedBundles
	 * 
	 * @param installedBundles
	 */
	private void uninstallConditionPlugins(List<Bundle> installedBundles) {

		for (Bundle bundle : installedBundles) {
			try {
				bundle.stop();
				bundle.uninstall();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		installedBundles.clear();

		firePropertyChange(PROP_UNINSTALLED_BUNDLES, null, null);
	}

	/**
	 * This methods looks for ConditionPlugins in the given directory.
	 * 
	 * The only way to get the ConditionPlugin for a new directory is installing
	 * the bundles and look for plugins with the specific extension get the name
	 * and then uninstall the plugins. (Its just a check).
	 * 
	 * @param path
	 * @return ConditionPlugins
	 */
	public Map<String, IConditionPlugin> checkForConditionPlugins(String path) {
		// uninstall old bundles, to NOT get old plugins with the extension
		uninstallConditionPlugins(installedBundles);

		// install the bundles from the new folder
		List<Bundle> temporaryInstalledBundles = installBundles(path);

		// get extension from the new bundles
		Map<String, IConditionPlugin> conditionPlugins = getConditionPlugins();

		// uninstall the bundles from the check
		uninstallConditionPlugins(temporaryInstalledBundles);

		// reinstall the original bundles
		String pluginPath = getPluginPath();
		installedBundles = installBundles(pluginPath);

		return conditionPlugins;

	}

	/**
	 * Returns the path of the Plugin Folder. This value is stored in the
	 * eclipse preferences
	 * 
	 * @return
	 */
	private String getPluginPath() {
		if (pluginPath == null) {
			pluginPath = Activator.getDefault().getPreferenceStore().getString(PreferenceConstants.PREF_CONDITION_PLUGIN_PATH);
		}
		return pluginPath;
	}

	/**
	 * Set the new pluginpath.
	 * 
	 * @param pluginPath
	 */
	public void setPluginPath(String pluginPath) {
		String oldPath = pluginPath;
		this.pluginPath = pluginPath;

		if (!pluginPath.equals(oldPath)) {
			firePropertyChange(PROP_SET_PLUGINPATH, oldPath, pluginPath);
		}
	}

	/**
	 * Fires a property change event with the given name.
	 * 
	 * @param name
	 * @param oldValue
	 * @param newValue
	 */
	public void firePropertyChange(String name, Object oldValue, Object newValue) {
		propertyChangeSupport.firePropertyChange(name, oldValue, newValue);
	}

	/**
	 * Add the Listener to get notified about changes .
	 * 
	 * @param propertyChangeListener
	 */
	public void addPropertyChangeListener(PropertyChangeListener propertyChangeListener) {
		propertyChangeSupport = new PropertyChangeSupport(this);

		propertyChangeSupport.addPropertyChangeListener(propertyChangeListener);
	}

	/**
	 * Removes given Listener.
	 * 
	 * @param propertyChangeListener
	 */
	public void removePropertyChangeListener(PropertyChangeListener propertyChangeListener) {
		if (propertyChangeSupport != null) {
			propertyChangeSupport.removePropertyChangeListener(propertyChangeListener);
		}
	}

	public IConditionPlugin getActiveConditionPlugin() {
		return activeConditionPlugin;
	}

	public void setActiveConditionPlugin(IConditionPlugin activeConditionPlugin) {
		this.activeConditionPlugin = activeConditionPlugin;
	}

	private List<Bundle> getInstalledBundles() {
		if (installedBundles == null) {
			installedBundles = new ArrayList<Bundle>();
		}

		return installedBundles;
	}
}