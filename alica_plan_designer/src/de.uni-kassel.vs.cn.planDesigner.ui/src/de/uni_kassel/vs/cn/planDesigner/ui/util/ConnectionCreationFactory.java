package de.uni_kassel.vs.cn.planDesigner.ui.util;

import org.eclipse.gef.requests.CreationFactory;

import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.EntryPointStateDummyConnection;
import de.uni_kassel.vs.cn.planDesigner.ui.dummyModelObjects.SynchedTransitionDummyConnection;

/**
 * This factory has the same task to do, as ModelCreationFactory.java. 
 * We just don't want to create a model object
 * @author Stephan Opfer
 *
 */
public class ConnectionCreationFactory implements CreationFactory {
	
	private Object template;
	
	public ConnectionCreationFactory(Object template) {
		this.template = template;
	}

	public Object getNewObject() {
		Object obj = null;
		if (template instanceof EntryPointStateDummyConnection) {
			obj = new EntryPointStateDummyConnection();
		} else if (template instanceof SynchedTransitionDummyConnection) {
			obj = new SynchedTransitionDummyConnection();
		}
		return obj;
	}

	public Object getObjectType() {
		return template;
	}

}
