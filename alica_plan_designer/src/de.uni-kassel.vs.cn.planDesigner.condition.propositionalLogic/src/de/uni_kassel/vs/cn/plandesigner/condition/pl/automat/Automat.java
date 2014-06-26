package de.uni_kassel.vs.cn.plandesigner.condition.pl.automat;

import java.util.HashMap;
import java.util.Map;
import java.util.Stack;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;

/**
 * Push Down Automat to check if a String is an element of the Language of
 * Propositional Logic Expressions
 * 
 * @author philipp
 * 
 */
public class Automat {

	public static final String KEY_STATE00 = "state00";
	public static final String KEY_STATE01 = "state01";
	public static final String KEY_STATE02 = "state02";
	public static final String KEY_STATE03 = "state03";
	public static final String KEY_STATE04 = "state04";
	public static final String KEY_STATE05 = "state05";

	private int errorPosition;

	private String statusMessage;

	private AbstractState startState;

	private Stack<String> stack;

	private Map<String, AbstractState> states;

	public Automat() {
		statusMessage = "";
		errorPosition = 0;

		states = new HashMap<String, AbstractState>();
		states.put(KEY_STATE00, new State00(this));
		states.put(KEY_STATE01, new State01(this));
		states.put(KEY_STATE02, new State02(this));
		states.put(KEY_STATE03, new State03(this));
		states.put(KEY_STATE04, new State04(this));
		states.put(KEY_STATE05, new State05(this));

		startState = getState(KEY_STATE00);
	}

	/**
	 * Validates the input which represents a formular and returns a
	 * {@link ValidationResult}-Instance holding the results.
	 * 
	 * @param input
	 * @return
	 */
	public ValidationResult getResult(String input) {
		boolean stateResult = startState.getResult(input, 0);
		boolean isStackEmpty = getStack().isEmpty();

		if (stateResult == true && isStackEmpty == false) {
			setStatusMessage(PluginConstants.STATUS_MISSING_CLOSED_BRACKET);
		}
		
		if(stateResult && isStackEmpty){
			setStatusMessage(PluginConstants.STATUS_FORMULAR_IS_VALID);
		}
		
		ValidationResult result = new ValidationResult((stateResult && isStackEmpty), getStatusMessage(), getErrorPosition());

		return result;
	}

	private int getErrorPosition() {
		return errorPosition;
	}

	public void setErrorPosition(int errorPosition) {
		this.errorPosition = errorPosition;
	}

	private String getStatusMessage() {
		return statusMessage;
	}

	public void setStatusMessage(String statusMessage) {
		this.statusMessage = statusMessage;
	}

	public Stack<String> getStack() {
		if (stack == null) {
			stack = new Stack();
		}
		return stack;
	}

	public AbstractState getState(String key) {
		return states.get(key);
	}

}
