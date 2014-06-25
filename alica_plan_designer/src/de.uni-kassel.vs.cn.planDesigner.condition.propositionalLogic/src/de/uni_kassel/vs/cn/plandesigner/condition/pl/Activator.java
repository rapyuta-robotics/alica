package de.uni_kassel.vs.cn.plandesigner.condition.pl;

import java.net.MalformedURLException;
import java.net.URL;

import org.eclipse.core.runtime.Platform;
import org.eclipse.jface.action.IContributionItem;
import org.eclipse.jface.action.IContributionManager;
import org.eclipse.jface.resource.ImageDescriptor;
import org.eclipse.jface.resource.ImageRegistry;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.CoolBar;
import org.eclipse.swt.widgets.Menu;
import org.eclipse.swt.widgets.ToolBar;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.internal.WorkbenchWindow;
import org.eclipse.ui.plugin.AbstractUIPlugin;
import org.osgi.framework.BundleContext;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.preferences.PLPluginPreferenceInitializer;

/**
 * The activator class controls the plug-in life cycle
 */
public class Activator extends AbstractUIPlugin {

	// The plug-in ID
	public static final String PLUGIN_ID = "de.uni-kassel.vs.cn.planDesigner.condition.propositionalLogic"; //$NON-NLS-1$

	// The shared instance
	private static Activator plugin;

	/**
	 * The constructor
	 */
	public Activator() {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * org.eclipse.ui.plugin.AbstractUIPlugin#start(org.osgi.framework.BundleContext
	 * )
	 */
	public void start(BundleContext context) throws Exception {
		super.start(context);
		plugin = this;
		// needed, because bundles are started at runtime from
		// ConditionPluginLoader, and so the preferences are not handled by
		// eclipse framework
		PLPluginPreferenceInitializer initializer = new PLPluginPreferenceInitializer();
		initializer.initializeDefaultPreferences();

		getImageRegistry();
		
	
		//
		// System.out.println("Activator.start(...).new Thread() {...}.run().new Runnable() {...}.run()");

		// IWorkbenchWindow window =
		// PlatformUI.getWorkbench().getActiveWorkbenchWindow();
		// IToolBarManager toolBarManager = ((WorkbenchWindow)
		// window).getActionBars().getToolBarManager();
		//
		// ToolBarContributionItem contribution = new
		// ToolBarContributionItem(toolBarManager);
		//
		//

	

		PlatformUI.getWorkbench().getDisplay().update();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * org.eclipse.ui.plugin.AbstractUIPlugin#stop(org.osgi.framework.BundleContext
	 * )
	 */
	public void stop(BundleContext context) throws Exception {
		plugin = null;
		super.stop(context);
	}

	/**
	 * Returns the shared instance
	 * 
	 * @return the shared instance
	 */
	public static Activator getDefault() {
		return plugin;
	}

	@Override
	protected void initializeImageRegistry(ImageRegistry registry) {
		super.initializeImageRegistry(registry);

		URL entry = getBundle().getEntry("/");

		URL url = null;
		try {
			url = new URL(entry, PluginConstants.ICON_FORMULAR_ADD_PATH);
			ImageDescriptor desc = ImageDescriptor.createFromURL(url);
			registry.put(PluginConstants.ICON_FORMULAR_ADD, desc);

			url = new URL(entry, PluginConstants.ICON_FORMULAR_PATH);
			desc = ImageDescriptor.createFromURL(url);
			registry.put(PluginConstants.ICON_FORMULAR, desc);

			url = new URL(entry, PluginConstants.ICON_PROPOSITION_PATH);
			desc = ImageDescriptor.createFromURL(url);
			registry.put(PluginConstants.ICON_PROPOSITION, desc);

			url = new URL(entry, PluginConstants.ICON_FAILURE_PATH);
			desc = ImageDescriptor.createFromURL(url);
			registry.put(PluginConstants.ICON_FAILURE, desc);

		} catch (MalformedURLException e) {
			e.printStackTrace();
		}

	}
}
