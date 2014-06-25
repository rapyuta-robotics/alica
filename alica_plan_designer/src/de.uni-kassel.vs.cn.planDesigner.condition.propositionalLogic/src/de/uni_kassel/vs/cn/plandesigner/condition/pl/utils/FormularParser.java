package de.uni_kassel.vs.cn.plandesigner.condition.pl.utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import org.eclipse.jface.preference.IPreferenceStore;

import de.uni_kassel.vs.cn.plandesigner.condition.core.Activator;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Formular;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Proposition;
import de.uni_kassel.vs.cn.plandesigner.condition.pl.model.Vocabulary;

public class FormularParser {

	/**
	 * Parses proposition ini and returns the vocabulary
	 * 
	 * @return
	 */
	public Vocabulary getPropositionVocabulary() {
		IPreferenceStore preferenceStore = Activator.getDefault().getPreferenceStore();
		String path = preferenceStore.getString(PluginConstants.PREFERENCE_PROPOSITION_VOCABULARY_PATH);
		Vocabulary vocabulary = parseVocabulary(path, true);
		vocabulary.setPath(path);

		return vocabulary;
	}

	public Vocabulary getLinkedFormularVocabulary() {
		IPreferenceStore preferenceStore = Activator.getDefault().getPreferenceStore();
		String path = preferenceStore.getString(PluginConstants.PREFERENCE_LINKED_FORMULAR_VOCABULARY_PATH);
		Vocabulary vocabulary = parseVocabulary(path, false);
		vocabulary.setPath(path);

		return vocabulary;
	}

	private Vocabulary parseVocabulary(String path, boolean propositionVocabulary) {
		Vocabulary vocabulary = new Vocabulary();

		BufferedReader reader = getBufferedReaderFromFile(path);
		String readedLine = null;

		if (reader == null) {
			System.out.println("Could not find vocabulary-file. Return empty vocabulary.");
			return new Vocabulary();
		}
		
		try {
			while ((readedLine = reader.readLine()) != null) {
				String[] lineSplit = readedLine.split("=");
				if (lineSplit.length < 2) {
					continue;
				}

				String name = lineSplit[0];
				String expression = lineSplit[1];

				Formular formular;

				if (propositionVocabulary) {
					formular = new Proposition();
				} else {
					formular = new Formular();
				}

				formular.setName(name);
				formular.setExpression(expression);

				vocabulary.addToFormulars(formular);
			}
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println("FormularParser: Problems while parsing the vocabularies.");
		}

		return vocabulary;
	}

	private BufferedReader getBufferedReaderFromFile(String path) {
		try {
			return new BufferedReader(new FileReader(new File(path)));
		} catch (Exception e) {
			return null;
		}
	}

}
