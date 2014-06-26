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
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.gef.GraphicalEditPart;

import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap;

public interface UIAwareEditor {
	
	/**
	 * Gets the UIExtension object for the given EObject. If <code>create</code>
	 * is true, a new UIExtension will be created if non exists.
	 * @param obj
	 * @param create
	 * @return
	 */
	PmlUiExtension getUIExtension(final EObject obj, boolean create);
	
	/**
	 * Returns the map which holds all UIExtensions.
	 * @return
	 */
	PmlUiExtensionMap getUIExtensionMap();
	
	/**
	 * Returns the resource which holds the UIExtension map.
	 * @return
	 */
	Resource getUIExtensionResource();
	
	GraphicalEditPart getRootEditPart();

}
