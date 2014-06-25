package de.uni_kassel.vs.cn.plandesigner.condition.pl;

public class PluginConstants {
	// Regular expressions to check formular input
	public static final String REGEX_OPERAND_SIGN = "[a-zA-Z0-9]";
	public static final String REGEX_OPERAND = "[a-zA-Z0-9]+";
	public static final String REGEX_BI_OPERATOR = "[&|]";
	public static final String REGEX_OPEN_BRACKET = "\\(";
	public static final String REGEX_CLOSED_BRACKET = "\\)";
	public static final String REGEX_WHITESPACE = "\\s";
	public static final String REGEX_MULTIWHITESPACE = "\\s+";
	public static final String REGEX_OPERATOR = "[!\\(\\)|&]";
	
	public static final String OPERATOR_AND = "&";
	public static final String OPERATOR_OR = "|";
	public static final String OPERATOR_NEG = "!";

	// Signs to check formular input
	public static final String SIGN_OPEN_BRACKET = "(";
	public static final String SIGN_CLOSED_BRACKET = ")";
	public static final String SIGN_UNI_OPERATOR = "!";
	public static final String SIGN_WHITESPACE = " ";

	//Status messages after checking formular input
	public static final String STATUS_INVALID_FORMULAR_NAME = "Invalid formular name.";
	
	public static final String STATUS_INVALID_SIGN = "Invalid sign at position: ";
	public static final String STATUS_MISSING_OPERAND = "Missing Operand.";
	public static final String STATUS_MISSING_CLOSED_BRACKET = "Missing closing bracket";
	public static final String STATUS_MISSING_EXPRESSION = "No formular expression";
	public static final String STATUS_MISSING_NAME = "Missing formular name";
	public static final String STATUS_FORBIDDEN_CIRCLE = "Formular contains itself. This is not allowed.";
	public static final String STATUS_INVALID_FILE_TYPE = "Invalid filetype. Must be a .txt";
	public static final String STATUS_FORMULAR_IS_VALID = "Formular is valid";
	public static final String STATUS_FORMULAR_NOT_VALID = "Formular not valid";	
	public static final String STATUS_FORMULAR_DUPLICATED = "Formular is already in the vocabulary";
	public static final String STATUS_OVERRIDE_FORMULAR = "Formular is already in the vocabulary. Do you want to override it?";
	public static final String STATUS_UNKOWN_FORMULAR = "Unkown formular: ";
	public static final String STATUS_PROPOSITION_IS_VALID = "Proposition is valid";
	public static final String STATUS_EVERY_FORMULAR_VALID = "Every formular is valid";
	public static final String STATUS_PROBLEMS_WITH_VOCABULARIES = "One of the vocabularies is empty. Is file of vocabulary still alive?";	
	
	//Property Change Messages
	public static final String PC_VOCABULARY_CHANGED = "pc_vocabulary_changed";
	
	//Preference Constants
	public static final String PREFERENCE_PROPOSITION_VOCABULARY_PATH = "preference_proposition_vocabulary_path";
	public static final String PREFERENCE_LINKED_FORMULAR_VOCABULARY_PATH = "preference_linked_formular_vocabulary_path";
	
	//Keys for Conditions pluginParameters Attribute
	public static final String PARAMETER_FORMULAR = "parameter_formular";
	public static final String PARAMETER_RESOLVED_FORMULAR = "parameter_resolved_formular";
	public static final String PARAMETER_RESOLVED_OPERANDS = "parameter_operands";
		
	public static final String ICON_FORMULAR_ADD = "formulaAdd16x16.png";
	public static final String ICON_FORMULAR_ADD_PATH = "icons/formulaAdd16x16.png";
	
	public static final String ICON_FORMULAR = "formula16x16.png";
	public static final String ICON_FORMULAR_PATH = "icons/formula16x16.png";
	
	public static final String ICON_PROPOSITION = "proposition16x16.png";
	public static final String ICON_PROPOSITION_PATH = "icons/proposition16x16.png";
	
	public static final String ICON_FAILURE = "failurePoint16x16.png";
	public static final String ICON_FAILURE_PATH = "icons/failurePoint16x16.png";
	
	public static final String HEADLINE_PROPERTIES = "Formular properties:";
	public static final String HEADLINE_REPOSITORY = "Formular Repository:\n(Choose one to edit.)";
	public static final String HEADLINE_WRONG_FORMULAR = "Something wrong with the formular";
	public static final String HEADLINE_PROBLEM_VOCABULARY = "Problems with vocabularies";
	
	public static final String UI_ADD = "Add";
	public static final String UI_REMOVE = "Remove";
	public static final String UI_NEW = "New";
	public static final String UI_VALIDATE = "Validate";
	public static final String UI_EDIT = "Edit";
	public static final String UI_SHOW_USAGE = "Show Usage";
	public static final String UI_TITLE_FORMULAR_EDITOR = "Formular Editor";
	
	
}
