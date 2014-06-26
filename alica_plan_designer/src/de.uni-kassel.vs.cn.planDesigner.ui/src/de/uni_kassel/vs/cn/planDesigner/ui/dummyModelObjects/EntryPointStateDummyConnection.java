package de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects;

import de.uni_kassel.vs.cn.planDesigner.alica.EntryPoint;
import de.uni_kassel.vs.cn.planDesigner.alica.State;

public class EntryPointStateDummyConnection {
	
	protected EntryPoint source;
	protected State target;
	
	public EntryPointStateDummyConnection () {
	}
	
	public EntryPointStateDummyConnection (EntryPoint source, State target) {
		this.source = source;
		this.target = target;
	}
	
	public EntryPoint getSource() {
		return this.source;
	}
	
	public void setSource (EntryPoint source) {
		this.source = source;
	}
	
	public State getTarget () {
		return this.target;
	}
	
	public void setTarget (State target) {
		this.target = target;
	}
	 
	@Override
	public boolean equals(Object obj) {
		if (obj instanceof EntryPointStateDummyConnection) {
			EntryPointStateDummyConnection esdc = (EntryPointStateDummyConnection) obj;
			if (esdc.getSource() == this.source && esdc.getTarget() == this.target) {
				return true;
			}
		}
		return false;
	}
	
	@Override
	public int hashCode() {
		int hash = this.source.hashCode();
		hash |= this.target.hashCode();
		return hash;
	}

}
