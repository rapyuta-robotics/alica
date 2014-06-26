package de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects;

import de.uni_kassel.vs.cn.planDesigner.alica.Synchronisation;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;

public class SynchedTransitionDummyConnection {
	
	protected Synchronisation source;
	protected Transition target;
	
	public SynchedTransitionDummyConnection () {
	}
	
	public SynchedTransitionDummyConnection (Synchronisation source, Transition target) {
		this.source = source;
		this.target = target;
	}
	
	public Synchronisation getSource() {
		return this.source;
	}
	
	public void setSource (Synchronisation source) {
		this.source = source;
	}
	
	public Transition getTarget () {
		return this.target;
	}
	
	public void setTarget (Transition target) {
		this.target = target;
	}
	
	@Override
	public boolean equals(Object obj) {
		if (obj instanceof SynchedTransitionDummyConnection) {
			SynchedTransitionDummyConnection stdc = (SynchedTransitionDummyConnection) obj;
			if (stdc.getSource() == this.source && stdc.getTarget() == this.target) {
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
