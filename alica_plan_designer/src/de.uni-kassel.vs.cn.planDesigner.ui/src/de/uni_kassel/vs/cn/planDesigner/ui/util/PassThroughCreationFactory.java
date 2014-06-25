// Copyright 2009 Distributed Systems Group, University of Kassel
// This program is distributed under the GNU Lesser General Public License (LGPL).
//
// This file is part of the Carpe Noctem Software Framework.
//
//    The Carpe Noctem Software Framework is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Lesser General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    The Carpe Noctem Software Framework is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Lesser General Public License for more details.
package de.uni_kassel.vs.cn.planDesigner.ui.util;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.gef.requests.CreationFactory;

/**
 * This factory doesn't create object itself. It just passes an object which
 * is already there to someone who wants it e.g. to CreateRequests.
 * @author Zenobios
 *
 */
public class PassThroughCreationFactory implements CreationFactory {
	
	private EObject theObject;
	
	public PassThroughCreationFactory(EObject theObject) {
		this.theObject = theObject;
	}

	public Object getNewObject() {
		return theObject;
	}

	public Object getObjectType() {
		return theObject.eClass();
	}

}
