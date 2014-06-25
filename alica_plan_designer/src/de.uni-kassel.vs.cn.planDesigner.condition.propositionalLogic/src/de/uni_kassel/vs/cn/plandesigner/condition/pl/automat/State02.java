package de.uni_kassel.vs.cn.plandesigner.condition.pl.automat;

import java.util.regex.Pattern;

import de.uni_kassel.vs.cn.plandesigner.condition.pl.PluginConstants;

public class State02 extends AbstractState {

	public State02(Automat automat) {
		super(automat);
	}

	@Override
	public boolean getResult(String input, int position) {
//		System.out.println("State 2");
		if (position >= input.length()) {
			// TODO errormessage
			getAutomat().setStatusMessage(PluginConstants.STATUS_MISSING_OPERAND);
			return false;
		}

		String signToCheck = getSignToCheck(input, position);
		if(PluginConstants.SIGN_WHITESPACE.equals(signToCheck)){
			return getResult(input, ++position);
		}else if(PluginConstants.SIGN_OPEN_BRACKET.equals(signToCheck)){
			getAutomat().getStack().push(PluginConstants.SIGN_OPEN_BRACKET);
			return getResult(input, ++position);
		}else if(Pattern.matches(PluginConstants.REGEX_OPERAND_SIGN, signToCheck)){
			return getAutomat().getState(Automat.KEY_STATE01).getResult(input, ++position);
		}else if(PluginConstants.SIGN_UNI_OPERATOR.equals(signToCheck)){
			return getResult(input, ++position);
		}else{
			//TODO richtige errormessage
			getAutomat().setStatusMessage(PluginConstants.STATUS_INVALID_SIGN + ++position);
			getAutomat().setErrorPosition(++position);
			return false;
		}
	}

}
