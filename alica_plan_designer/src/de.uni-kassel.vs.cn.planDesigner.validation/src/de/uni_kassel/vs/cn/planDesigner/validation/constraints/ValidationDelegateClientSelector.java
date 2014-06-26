package de.uni_kassel.vs.cn.planDesigner.validation.constraints;

import org.eclipse.emf.validation.model.IClientSelector;

//NOTE: This is _NOT_ a recommended approach to writing a client selector.
//Suggested approaches:
//  -Check the resource of the EObject either by identity or by URI
//   as long as this resource is somehow unique to this application
//  -Check the identity of the resource set to ensure that it is some
//   private object
//  -Check the identity of the EObject itself to see if it belongs to
//   some private collection
//  -Check the EClass of the EObject but only if the metamodel is private
//   to this application and will not be used by other contexts
public class ValidationDelegateClientSelector implements IClientSelector {

public static boolean running = true;

	public boolean selects(Object object) {
		//System.out.println("Delegate Client Selector "+object);
		return running;
	}
}
