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
package de.uni_kassel.vs.cn.planDesigner.ui.policies;

import org.eclipse.draw2d.geometry.Rectangle;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.gef.EditPart;
import org.eclipse.gef.commands.Command;
import org.eclipse.gef.editpolicies.XYLayoutEditPolicy;
import org.eclipse.gef.requests.CreateRequest;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.CommandWrap4EMF;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.SetUIExtensionCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.parts.AbstractElementEditPart;
import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.UIAwareEditor;

/**
 * A XYLayoutEditPolicy which can modify the positions of elements but is NOT
 * able to create new elements!
 * @author Zenobios
 *
 */
public class DesignerXYLayoutEditPolicy extends XYLayoutEditPolicy {

	@Override
	protected Command createChangeConstraintCommand(EditPart child,
			Object constraint) {
		EObject model = ((AbstractElementEditPart)child).getEObjectModel();
		Rectangle rectConstr = (Rectangle)constraint;
		UIAwareEditor editor = CommonUtils.getUIAwareEditorAdapter(getHost());
		TransactionalEditingDomain editingDomain =  CommonUtils.getEditingDomainAdapter(getHost());
		return new CommandWrap4EMF(new SetUIExtensionCommand(model, rectConstr, editingDomain,editor.getUIExtensionMap()));
	}

	@Override
	protected Command getCreateCommand(CreateRequest request) {
		return null;
	}

}
