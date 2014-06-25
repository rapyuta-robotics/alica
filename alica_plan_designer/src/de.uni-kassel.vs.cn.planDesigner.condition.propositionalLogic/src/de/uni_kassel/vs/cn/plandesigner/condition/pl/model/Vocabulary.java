package de.uni_kassel.vs.cn.plandesigner.condition.pl.model;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;

public class Vocabulary {
	/**
	 * Map with the formulars
	 */
	private Map<String, Formular> formulars;
	/**
	 * Path in filesystem to the vocabulary
	 */
	private String path;

	private PropertyChangeSupport propertyChangeSupport;

	// /**
	// * Property Change Message if formular is added
	// */
	// private String addPropertyChangeMessage;

	/**
	 * Adds formular to vocabulary. To persist the operation, call save()
	 * afterwards.
	 * 
	 * @param formular
	 */
	public void addToFormulars(Formular formular) {
		//set childs of the new formular		
//		PropositionalLogicPlugin.getInstance().addChildsToFormular(formular);
		getFormulars().put(formular.getName(), formular);

		if (getPropertyChangeSupport() != null) {
			getPropertyChangeSupport().firePropertyChange(PluginConstants.PC_VOCABULARY_CHANGED, null, formular);
		}
	}

	/**
	 * Removes the given formular from the vocabulary
	 * 
	 * @param formular
	 */
	public void removeFromFormulars(Formular formular) {
		getFormulars().remove(formular.getName());
	}

	/**
	 * Saves the vocabulary to file given in attribute path
	 * 
	 * @return
	 */
	public boolean save() {
		try {
			PrintWriter writer = new PrintWriter(new FileWriter(new File(path)));

			for (String key : getFormulars().keySet()) {
				Formular formular = getFormulars().get(key);
				writer.println(formular.getName() + "=" + formular.getExpression());
			}
			writer.close();
			return true;
		} catch (Exception e) {
			return false;
		}
	}

	public Formular getFormular(String name) {
		return getFormulars().get(name);
	}

	/**
	 * Returns weather or not the vocabulary is empty.
	 */
	public boolean isEmpty() {
		return getFormulars().isEmpty();
	}

	/**
	 * Returns all formularnames in the vocabulary
	 * 
	 * @return
	 */
	public List<String> getFormularNames() {
		ArrayList<String> list = new ArrayList<String>();
		for (String s : getFormulars().keySet()) {
			list.add(s);
		}

		Collections.sort(list);

		return list;
	}

	/**
	 * Returns true if vocabulary contains formular with given name.
	 * 
	 * @param formularName
	 * @return
	 */
	public boolean contains(String formularName) {
		return getFormulars().containsKey(formularName);
	}

	/**
	 * <pre>
	 *           1..1     1..1
	 * Vocabulary ------------------------- PropositionalLogicPlugin
	 *           propositionVocabulary        &lt;       propositionalLogicPlugin
	 * </pre>
	 */
	private PropositionalLogicPlugin propositionalLogicPlugin;

	public void setPropositionalLogicPlugin(PropositionalLogicPlugin value) {
		this.propositionalLogicPlugin = value;
	}

	public PropositionalLogicPlugin getPropositionalLogicPlugin() {
		return this.propositionalLogicPlugin;
	}

	private PropertyChangeSupport getPropertyChangeSupport() {
		return propertyChangeSupport;
	}

	public void addPropertyChangeListener(PropertyChangeListener listener) {
		if (getPropertyChangeSupport() != null) {
			getPropertyChangeSupport().addPropertyChangeListener(listener);
		}
	}

	public void removePropertyChangeListener(PropertyChangeListener listener) {
		if (getPropertyChangeSupport() != null) {
			getPropertyChangeSupport().removePropertyChangeListener(listener);
		}
	}

	public void setPropertyChangeSupport(PropertyChangeSupport propertyChangeSupport) {
		this.propertyChangeSupport = propertyChangeSupport;
	}

	private Map<String, Formular> getFormulars() {
		if (this.formulars == null) {
			this.formulars = new HashMap<String, Formular>();
		}

		return this.formulars;
	}

	public String getPath() {
		return path;
	}

	public void setPath(String path) {
		this.path = path;
	}

	/**
	 * Checks if parameter is an instance of formular or proposition and returns
	 * the belonging vocabulary.
	 * 
	 * @param f
	 */
	public static Vocabulary getCorrectVocabulary(Formular f) {
		if (f instanceof Proposition) {
			return PropositionalLogicPlugin.getInstance().getPropositionVocabulary();
		} else {
			return PropositionalLogicPlugin.getInstance().getLinkedFormularsVocabulary();
		}
	}

	/**
	 * If parameter is instance of proposition, f is removed from proposition
	 * vocabulary, otherwise from the linked formular vocabulary.
	 * 
	 * @param f
	 *            to remove.
	 */
	public static void removeFromCorrectVocabulary(Formular f) {
		Vocabulary v = getCorrectVocabulary(f);
		v.removeFromFormulars(f);
		v.save();
	}

}
