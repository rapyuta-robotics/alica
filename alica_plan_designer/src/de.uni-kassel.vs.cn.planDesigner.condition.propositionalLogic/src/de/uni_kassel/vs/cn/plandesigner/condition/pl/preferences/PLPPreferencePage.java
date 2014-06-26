package de.uni_kassel.vs.cn.plandesigner.condition.pl.preferences;

import java.util.HashMap;
import java.util.Map;

import org.eclipse.jface.preference.IPreferenceStore;
import org.eclipse.jface.preference.PreferencePage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.FileDialog;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;
import org.eclipse.ui.IWorkbench;
import org.eclipse.ui.IWorkbenchPreferencePage;

import de.uni_kassel.vs.cn.plandesigner.condition.core.Activator;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.PropositionalLogicPlugin;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Vocabulary;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.utils.FormularParser;

public class PLPPreferencePage extends PreferencePage implements IWorkbenchPreferencePage {
	/**
	 * Map to store the selected preferences values.
	 */
	private Map<String, String> preferenceValues;
	
	/**
	 * Map which contains informations which preference is valid
	 */
	private Map<String, Boolean> preferenceIsValid;
	
	

	@Override
	public void init(IWorkbench workbench) {
		preferenceValues = new HashMap<String, String>();
		preferenceIsValid = new HashMap<String, Boolean>();
		
		IPreferenceStore store = Activator.getDefault().getPreferenceStore();
		setPreferenceStore(store);

		String linkedPath = store.getString(PluginConstants.PREFERENCE_LINKED_FORMULAR_VOCABULARY_PATH);
		String propositionPath = store.getString(PluginConstants.PREFERENCE_PROPOSITION_VOCABULARY_PATH);

		// current preference values
		preferenceValues.put(PluginConstants.PREFERENCE_LINKED_FORMULAR_VOCABULARY_PATH, linkedPath);
		preferenceValues.put(PluginConstants.PREFERENCE_PROPOSITION_VOCABULARY_PATH, propositionPath);
		
		preferenceIsValid.put(PluginConstants.PREFERENCE_LINKED_FORMULAR_VOCABULARY_PATH, new Boolean(true));
		preferenceIsValid.put(PluginConstants.PREFERENCE_PROPOSITION_VOCABULARY_PATH, new Boolean(true));

	}
	
	@Override
	protected void performApply() {
		performOk();
	}

	@Override
	protected Control createContents(Composite parent) {
		Group container = new Group(parent, SWT.NONE);
		container.setLayout(new GridLayout(2, false));
		container.setLayoutData(new GridData(SWT.FILL, SWT.CENTER, true, false));
		container.setText("Requiered settings");

		//set propositions file
		Label pathPropositionLabel = new Label(container, SWT.NONE);
		pathPropositionLabel.setText("Path to proposition vocabulary:");
		new Label(container, SWT.NONE);

		Text textPropositionPath = new Text(container, SWT.BORDER | SWT.READ_ONLY);
		textPropositionPath.setLayoutData(new GridData(SWT.FILL, SWT.CENTER, true, false, 1, 1));

		Button propositionBrowseButton = new Button(container, SWT.NONE);
		propositionBrowseButton.setText("Browse");

		//set linked formulars file
		Label pathLabel = new Label(container, SWT.NONE);
		pathLabel.setText("Path to linked formulars vocabulary:");
		new Label(container, SWT.NONE);

		Text textLinkedFormulars = new Text(container, SWT.BORDER | SWT.READ_ONLY);
		textLinkedFormulars.setLayoutData(new GridData(SWT.FILL, SWT.CENTER, true, false, 1, 1));

		Button browseButton = new Button(container, SWT.NONE);
		browseButton.setText("Browse");
		
		Label statusLabel = new Label(container, SWT.NONE);
		statusLabel.setText("");
		statusLabel.setVisible(false);
		new Label(container, SWT.NONE);

		//set with values
		textPropositionPath.setText(preferenceValues.get(PluginConstants.PREFERENCE_PROPOSITION_VOCABULARY_PATH));
		textLinkedFormulars.setText(preferenceValues.get(PluginConstants.PREFERENCE_LINKED_FORMULAR_VOCABULARY_PATH));

		//init listeners
		propositionBrowseButton.addSelectionListener(new SetVocabularyPathListener(PluginConstants.PREFERENCE_PROPOSITION_VOCABULARY_PATH, textPropositionPath, statusLabel));
		browseButton.addSelectionListener(new SetVocabularyPathListener(PluginConstants.PREFERENCE_LINKED_FORMULAR_VOCABULARY_PATH, textLinkedFormulars, statusLabel));
		
		
		
		return container;
	}
	


	@Override
	public boolean performOk() {
		// commit values
		IPreferenceStore preferenceStore = Activator.getDefault().getPreferenceStore();
		for (String key : preferenceValues.keySet()) {
			preferenceStore.putValue(key, preferenceValues.get(key));
		}
		
		
		
		//set the new vocabulary (parser uses preference values)
		FormularParser parser = new FormularParser();
		Vocabulary v = parser.getLinkedFormularVocabulary();
		PropositionalLogicPlugin.getInstance().setLinkedFormularsVocabulary(v);
		
		v = parser.getPropositionVocabulary();
		PropositionalLogicPlugin.getInstance().setPropositionVocabulary(v);
		
		return true;
	}

	private class SetVocabularyPathListener implements SelectionListener {
		private String preferenceName;
		private Text text;
		private Label statusLabel;

		public SetVocabularyPathListener(String preferenceName, Text text, Label statusLabel) {
			this.preferenceName = preferenceName;
			this.text = text;
			this.statusLabel = statusLabel;
		}

		@Override
		public void widgetSelected(SelectionEvent e) {
			
			FileDialog dialog = new FileDialog(getShell());
			String file = dialog.open();
			if (null != file && !"".equals(file) && file.contains(".txt")) {
				preferenceValues.put(preferenceName, file);
				text.setText(file);
				statusLabel.setText("");
				statusLabel.update();
				
				setPreferenceValid(preferenceName, true);
			} else if(null != file && ("".equals(file) || !file.contains(".txt"))) {
				text.setText(file);
				statusLabel.setText(PluginConstants.STATUS_INVALID_FILE_TYPE);
				statusLabel.update();
				setPreferenceValid(preferenceName, false);
			}

		}

		@Override
		public void widgetDefaultSelected(SelectionEvent e) {
			//NOTHING TO DO
		}

	}

	/**
	 * Sets if preference is valid or not, and updates the validation status of the preference state
	 * @param name
	 * @param b
	 */
	private void setPreferenceValid(String name, Boolean b){
		preferenceIsValid.put(name, b);
		
		boolean isValid = true;
		for(String preference : preferenceIsValid.keySet()){
			if(preferenceIsValid.get(preference) == null || !preferenceIsValid.get(preference)){
				isValid = false;
			}
		}
		
		setValid(isValid);
	}
}
