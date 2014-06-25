package de.uni_kassel.vs.cn.plandesigner.condition.core.ui;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.util.Set;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.CCombo;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.MessageBox;
import org.eclipse.swt.widgets.TypedListener;
import org.eclipse.ui.IWorkbenchPart;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.PMLPropertySection;
import de.uni_kassel.vs.cn.plandesigner.condition.core.Activator;
import de.uni_kassel.vs.cn.plandesigner.condition.core.ConditionPluginLoader;
import de.uni_kassel.vs.cn.plandesigner.condition.core.IConditionPlugin;
import de.uni_kassel.vs.cn.plandesigner.condition.core.preferences.PreferenceConstants;

public class ConditionPropertySection extends PMLPropertySection {
	/**
	 * The combo which show plugins.
	 */
	private CCombo pluginCombo;

	/**
	 * Content Composite where plugin adds his elements.
	 */
	private Composite pluginContentComposite;

	private Composite pluginSettingsComposite;

	/**
	 * Status Label to inform user about different things.
	 */
	private Label statusLabel;

	private Composite parent;

	private TabbedPropertySheetPage tabbedPropertySheetPage;

	@Override
	public void createControls(Composite parent, TabbedPropertySheetPage tabbedPropertySheetPage) {
		parent.setLayout(new GridLayout(1, false));
		super.createControls(parent, tabbedPropertySheetPage);
		this.parent = parent;

		// Create UI
		Group group = getWidgetFactory().createGroup(parent, "Plugin Settings");
		group.setLayout(new FillLayout(SWT.HORIZONTAL));
		group.setLayoutData(new GridData(SWT.FILL, SWT.TOP, true, false, 1, 1));

		pluginSettingsComposite = getWidgetFactory().createFlatFormComposite(group);
		pluginSettingsComposite.setLayout(new GridLayout(3, false));

		CCombo pluginCombo = getWidgetFactory().createCCombo(pluginSettingsComposite, SWT.NONE);
		GridData gd_pluginCombo = new GridData(SWT.LEFT, SWT.CENTER, false, false, 1, 1);
		gd_pluginCombo.widthHint = 234;
		pluginCombo.setLayoutData(gd_pluginCombo);
		new Label(pluginSettingsComposite, SWT.NONE);

//		Button refreshPluginsButton = getWidgetFactory().createButton(pluginSettingsComposite, "Refresh Plugins", SWT.NONE);

		group = getWidgetFactory().createGroup(parent, "Condition Plugin");
		group.setLayout(new FillLayout(SWT.HORIZONTAL));
		group.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true, 1, 1));

		Composite pluginContentComposite = getWidgetFactory().createFlatFormComposite(group);

		Label label = getWidgetFactory().createLabel(pluginContentComposite, "No Condition Plugin");

		Label statusLabel = getWidgetFactory().createLabel(parent, "", SWT.NONE);
		statusLabel.setVisible(false);

		// set listener
//		refreshPluginsButton.addSelectionListener(new RefreshPluginsListener(pluginCombo));

		// save references to frequently UI Elements
		setPluginContentComposite(pluginContentComposite);
		setTabbedPropertySheetPage(tabbedPropertySheetPage);
		setPluginCombo(pluginCombo);
		setStatusLabel(statusLabel);

		// register listener to changes with the Condition Plugins
		ConditionPluginLoader.getInstance().addPropertyChangeListener(new RefreshedPluginListener());

		// INITIALIZATION OF PLUGINUI AFTER setInput()

	}

	@Override
	public void setInput(IWorkbenchPart part, ISelection selection) {
		System.out.println("ConditionPropertySection.setInput()");
		super.setInput(part, selection);

		// we need the setted input to init the combo.

		// get last or default Condition Plugin for the selected condition
		IConditionPlugin conditionPlugin = getConditionPlugin();
		if (conditionPlugin != null) {
			pluginContentComposite.setVisible(true);
			pluginSettingsComposite.setVisible(true);
			loadConditionPlugin(conditionPlugin);
		} else if (getCondition() == null) {
			// special case "red transitions"
			showStatusMessage("Transition has no condition");
			pluginContentComposite.setVisible(false);
			pluginSettingsComposite.setVisible(false);

		} else {
			pluginContentComposite.setVisible(true);
			pluginSettingsComposite.setVisible(true);
			unloadConditionPlugin();
			String pluginName = getCondition().getPluginName();
			if (null == pluginName || "".equals(pluginName)) {
				showStatusMessage("Default Plugin not in pluginfolder");
			} else {
				showStatusMessage("Missing Plugin: " + pluginName);
			}
			reLayoutView();
		}

		// init ui elements
		initCombo(getPluginCombo(), ConditionPluginLoader.getInstance().getConditionPluginNames(), conditionPlugin);
	}

	/**
	 * Returns the selected Condition.
	 * 
	 * @return
	 */
	private Condition getCondition() {
		EObject selection = getModel();
		if (selection instanceof Transition) {
			return ((Transition) selection).getPreCondition();
		} else {
			// we know that only a Condition can be selected here
			return ((Condition) selection);
		}
	}

	@Override
	protected void updateView(Notification n) {

	}

	/**
	 * 
	 * @param combo
	 *            combo to init
	 * @param values
	 *            values to init combo
	 * @param selectElement
	 *            controls if current plugin should be selcted in combo
	 */
	private void initCombo(CCombo combo, Set<String> values, IConditionPlugin conditionPlugin) {
		if (combo.isDisposed()) {
			return;
		}

		combo.removeAll();
		for (String value : values) {
			combo.add(value);
		}

		// remove the old instance of the ChangePluginSelectionListener
		Listener[] listeners = combo.getListeners(SWT.Selection);

		for (Listener listener : listeners) {
			if (listener instanceof TypedListener) {
				SelectionListener toTest = (SelectionListener) ((TypedListener) listener).getEventListener();
				if (toTest instanceof ChangePluginSelectionListener) {
					combo.removeSelectionListener(toTest);
					break;
				}
			}
		}

		// register listener
		ChangePluginSelectionListener changePluginSelectionListener = new ChangePluginSelectionListener();

		// if condition plugin is in folder, select it in the combo
		if (conditionPlugin != null) {
			String[] items = combo.getItems();

			for (int i = 0; i < items.length; ++i) {
				String item = items[i];
				if (item.equals(conditionPlugin.getName())) {
					combo.select(i);
					changePluginSelectionListener.setLastSelecetion(i);
					break;
				}
			}
		}
		combo.addSelectionListener(changePluginSelectionListener);
	}

	/**
	 * Checks if a Condition Plugin Name is stored in the selected Condition.
	 * Otherwise it will look in the eclipse preference to get the default
	 * Condition Plugin and return it.
	 * 
	 * If the Condition Plugin stored in the Condtion is not available anymore
	 * this method returns null.
	 * 
	 * @return
	 */
	private IConditionPlugin getConditionPlugin() {
		Condition condition = getCondition();
		if (condition == null) {
			// e.g. the "red transitions" have no conditions
			return null;
		}

		String pluginName = condition.getPluginName();
		if (pluginName != null && !"".equals(pluginName)) {
			IConditionPlugin conditionPlugin = ConditionPluginLoader.getInstance().getConditionPlugin(pluginName);
			// null value possible
			return conditionPlugin;
		}

		// no its not, set the default condition plugin in condition and return
		// it
		pluginName = Activator.getDefault().getPreferenceStore().getString(PreferenceConstants.PREF_DEFAULT_CONDITION_PLUGIN);
		IConditionPlugin defaultConditionPlugin = ConditionPluginLoader.getInstance().getConditionPlugin(pluginName);
		return defaultConditionPlugin;
	}

	/**
	 * Stores the last selected Condition Plugin in the Condition-Instance.
	 * 
	 * @param conditionPlugin
	 */
	private void setConditionPluginInCondition(final IConditionPlugin conditionPlugin) {
		getCommandStack().execute(new RecordingCommand(getEditingDomain()) {

			@Override
			protected void doExecute() {
				getCondition().setPluginName(conditionPlugin.getName());

			}

		});
	}

	/**
	 * Removes the UI Elements loaded by a Condition Plugin and returns the
	 * cleared Composite
	 * 
	 * @param contentComposite
	 */
	private void removePluginContent() {
		Composite contentComposite = getPluginContentComposite();

		Composite parent = contentComposite.getParent();
		contentComposite.dispose();
		contentComposite = getWidgetFactory().createFlatFormComposite(parent);
		contentComposite.setLayout(new FillLayout());

		// save the reference to the new content composite
		setPluginContentComposite(contentComposite);

	}

	/**
	 * Loads UI of a Condition Plugin
	 * 
	 * @param pluginName
	 * @param contentComposite
	 * @param tabbedPropertySheetPage
	 */
	private void loadPluginContent(IConditionPlugin conditionPlugin) {
		// init plugin and get ui
		Composite pluginContentComposite = getPluginContentComposite();

		PMLPropertySection pluginPropertySection = conditionPlugin.getUi();
		pluginPropertySection.createControls(pluginContentComposite, getTabbedPropertySheetPage());

		// set selected element (condition) in the plugin
		ISelection selection = getSelection();
		pluginPropertySection.setInput(getPart(), selection);

		reLayoutView();

	}

	/**
	 * Performs a complete relayout
	 */
	private void reLayoutView() {
		// relayout the complete UI
		Composite pluginContentComposite = getPluginContentComposite();
		pluginContentComposite.layout();

		Composite parentComposite = pluginContentComposite.getParent();
		while (parentComposite != null) {
			parentComposite.update();
			parentComposite.layout();
			parentComposite = parentComposite.getParent();
		}
	}

	/**
	 * Loads the given Condition Plugin.
	 * 
	 * @param conditionPlugin
	 */
	private void loadConditionPlugin(IConditionPlugin conditionPlugin) {
		removePluginContent();
		loadPluginContent(conditionPlugin);
		ConditionPluginLoader.getInstance().setActiveConditionPlugin(conditionPlugin);
		setConditionPluginInCondition(conditionPlugin);

	}

	private void unloadConditionPlugin() {
		removePluginContent();
		ConditionPluginLoader.getInstance().setActiveConditionPlugin(null);
	}

	/**
	 * Shows status label with text.
	 * 
	 * @param text
	 */
	private void showStatusMessage(String text) {
		if (getStatusLabel().isDisposed()) {
			return;
		}
		getStatusLabel().setVisible(true);
		getStatusLabel().setText(text);
		getStatusLabel().update();
		getStatusLabel().redraw();
	}

	/**
	 * Hides the status label
	 */
	private void hideStatusMessage() {
		if (getStatusLabel().isDisposed()) {
			return;
		}

		getStatusLabel().setVisible(false);
	}

	// Getter/Setter

	public CCombo getPluginCombo() {
		return pluginCombo;
	}

	public Composite getPluginContentComposite() {
		return pluginContentComposite;
	}

	public TabbedPropertySheetPage getTabbedPropertySheetPage() {
		return tabbedPropertySheetPage;
	}

	public void setPluginCombo(CCombo pluginCombo) {
		this.pluginCombo = pluginCombo;
	}

	public void setPluginContentComposite(Composite pluginContentComposite) {
		this.pluginContentComposite = pluginContentComposite;
	}

	public void setTabbedPropertySheetPage(TabbedPropertySheetPage tabbedPropertySheetPage) {
		this.tabbedPropertySheetPage = tabbedPropertySheetPage;
	}

	public Label getStatusLabel() {
		return statusLabel;
	}

	public void setStatusLabel(Label statusLabel) {
		this.statusLabel = statusLabel;
	}

//	/**
//	 * Tells ConditionPluginLoader to look in the plugin folder for new
//	 * Condition Plugins
//	 * 
//	 * @author philipp
//	 * 
//	 */
//	private class RefreshPluginsListener implements SelectionListener {
//		private CCombo pluginCombo;
//
//		public RefreshPluginsListener(CCombo pluginCombo) {
//			this.pluginCombo = pluginCombo;
//		}
//
//		@Override
//		public void widgetSelected(SelectionEvent e) {
//			ConditionPluginLoader.getInstance().refreshConditionPlugins();
//		}
//
//		@Override
//		public void widgetDefaultSelected(SelectionEvent e) {
//			// NOTHING TO DO
//		}
//
//	}

	/**
	 * This SelectionListener Implementation loads the new ConditionPlugin if
	 * the User chooses a new one and updates the UI with the new elements from
	 * the ConditionPlugin.
	 * 
	 * @author philipp
	 * 
	 */
	private class ChangePluginSelectionListener extends SelectionAdapter {
		/**
		 * Save last confirmed selection, to restore the selection if user
		 * dont't confirm the change via the MessageBox.
		 */
		private int lastSelection = -1;

		@Override
		public void widgetSelected(SelectionEvent e) {
			CCombo pluginCombo = getPluginCombo();
			if (pluginCombo.getSelectionIndex() == lastSelection) {
				// No new selection
				return;
			}

			// dialog for asking user if he really wants to change
			MessageBox confirmBox = new MessageBox(getPluginContentComposite().getShell(), SWT.ICON_ERROR | SWT.OK | SWT.CANCEL);
			confirmBox.setText("Condition Plugin Settings");
			confirmBox.setMessage("Do you really want to change the Condition Plugin? All handcoded C#-code will be removed after generation!");

			int result = confirmBox.open();

			if (result == SWT.OK) {
				// just in case it was shown before
				hideStatusMessage();

				// change condition plugin
				String pluginName = pluginCombo.getItem(pluginCombo.getSelectionIndex());
				IConditionPlugin conditionPlugin = ConditionPluginLoader.getInstance().getConditionPlugin(pluginName);

				loadConditionPlugin(conditionPlugin);

				setLastSelecetion(pluginCombo.getSelectionIndex());
			} else if (result == SWT.CANCEL) {
				// restore old selection in combo
				if (getLastSelection() != -1) {
					pluginCombo.select(getLastSelection());
				} else {
					pluginCombo.deselectAll();
				}
			}

		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {
			// NOTHING TO DO
		}

		private int getLastSelection() {
			return lastSelection;
		}

		private void setLastSelecetion(int lastSelection) {
			this.lastSelection = lastSelection;
		}
	}

	/**
	 * PropertyChangeListener to get notified if the Condition Plugins where
	 * refreshed.
	 * 
	 * @author philipp
	 * 
	 */
	private class RefreshedPluginListener implements PropertyChangeListener {
		/*
		 * 1) Plugins werden refreshed, verwendetes Plugin ist nicht mehr im
		 * Pluginfolder -> Combobox ist nichts selektiert -> Statusmeldung an
		 * Benutzer, das wechsel des Plugins nun dazu führt, das alte
		 * Modellierung überschrieben wirdPropositionalLogicPlugin
		 * 
		 * 2) System ist im Zustand von 1) Plugins werden refreshend,
		 * verwendetes Plugin ist nun im Pluginfolder und wird aktuell NICHT
		 * verwendet -> vollständiges laden des Plugins
		 * 
		 * 3) Plugins werden refreshed, verwendetes Plugin ist immernoch im
		 * Pluginfolder -> nichts passiert
		 */

		@Override
		public void propertyChange(PropertyChangeEvent evt) {
			if (ConditionPluginLoader.PROP_REFRESHED_PLUGINS.equals(evt.getPropertyName())) {
				if (!ConditionPropertySection.this.isEditable()) {
					return;
				}

				//plugin in der condition
				IConditionPlugin neededPlugin = getConditionPlugin();
				IConditionPlugin currentActivePlugin = ConditionPluginLoader.getInstance().getActiveConditionPlugin();

				// needed plugin is not availabe anymore
				if (neededPlugin == null) {
					unloadConditionPlugin();
					if (null == getCondition().getPluginName() || "".equals(getCondition().getPluginName())) {
						// default plugin not in pluginfolder
						showStatusMessage("Default Plugin not in plugin folder");
					} else {
						showStatusMessage("Missing Plugin: " + getCondition().getPluginName());
					}
					reLayoutView();
				} else if (currentActivePlugin == null) {
					//es gibt das conditionplugin in condition und momentan ist kein plugin aktiv
					hideStatusMessage();
					loadConditionPlugin(neededPlugin);
				}else if(neededPlugin.getName().equals(currentActivePlugin.getName())){
					//new version of the plugin
					hideStatusMessage();
					loadConditionPlugin(neededPlugin);
				}

				initCombo(getPluginCombo(), ConditionPluginLoader.getInstance().getConditionPluginNames(), neededPlugin);
			}

		}
	}

}
