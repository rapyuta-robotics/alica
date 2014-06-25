package de.uni_kassel.vs.cn.plandesigner.condition.pl.automat;

public abstract class AbstractState {
	private Automat automat;
	
	public AbstractState(Automat automat){
		this.automat = automat;
	}
	
	public abstract boolean getResult(String input, int position);

	protected String getSignToCheck(String input, int position){
		return input.charAt(position) + "";
	}

	public Automat getAutomat() {
		return automat;
	}

	public void setAutomat(Automat automat) {
		this.automat = automat;
	}
	
}
