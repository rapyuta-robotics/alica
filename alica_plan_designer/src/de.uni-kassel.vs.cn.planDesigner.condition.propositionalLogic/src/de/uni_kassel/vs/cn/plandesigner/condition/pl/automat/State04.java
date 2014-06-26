package de.uni_kassel.vs.cn.plandesigner.condition.pl.automat;

import java.util.regex.Pattern;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;

public class State04 extends AbstractState {

	public State04(Automat automat) {
		super(automat);
	}

	@Override
	public boolean getResult(String input, int position) {
//		System.out.println("State 4");
		if (position >= input.length()) {
//			getAutomat().setStatusMessage(Automat.ERROR_MISSING_OPERAND);
			return true;
		}

		String signToCheck = getSignToCheck(input, position);
		if (PluginConstants.SIGN_WHITESPACE.equals(signToCheck)) {
			return getResult(input, ++position);
		} else if (PluginConstants.SIGN_CLOSED_BRACKET.equals(signToCheck) && !getAutomat().getStack().isEmpty() && getAutomat().getStack().pop().equals(PluginConstants.SIGN_OPEN_BRACKET)) {
			return getAutomat().getState(Automat.KEY_STATE05).getResult(input, ++position);
		} else if (Pattern.matches(PluginConstants.REGEX_BI_OPERATOR, signToCheck)) {
			return getAutomat().getState(Automat.KEY_STATE00).getResult(input, ++position);
		} else {
			getAutomat().setStatusMessage(PluginConstants.STATUS_INVALID_SIGN + position);
			getAutomat().setErrorPosition(position);
			return false;	
		}
	}

}
