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
package de.uni_kassel.vs.cn.planDesigner.ui.commands;

import java.util.Collection;
import java.util.Collections;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.TransactionalEditingDomain;

import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelFactory;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtensionMap;

public class CreateUIExtensionCommmand extends RecordingCommand {
	
	
	private PmlUiExtensionMap map;
	
	private EObject object;
	
	private PmlUiExtension createdExtension;

	public CreateUIExtensionCommmand(TransactionalEditingDomain domain, PmlUiExtensionMap map, EObject object) {
		super(domain);
		this.map = map;
		this.object = object;
	}

	@Override
	protected void doExecute() {
		createdExtension = PmlUIExtensionModelFactory.eINSTANCE.createPmlUiExtension();
		
		map.getExtension().put(object, createdExtension);
		
	}

	@Override
	public Collection<?> getResult() {
		return Collections.singleton(createdExtension);
	}
}
