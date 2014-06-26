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

import java.util.HashMap;
import java.util.Map;

import org.eclipse.core.resources.IFile;
import org.eclipse.gef.requests.CreationFactory;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;

public class ResourceCreationFactory implements CreationFactory {
	
	private PMLTransactionalEditingDomain editingDomain;
	
	private IFile fromFile;
	
	private Map<String, Object> mapping;
	
	public ResourceCreationFactory(PMLTransactionalEditingDomain editingDomain, IFile fromFile) {
		this.editingDomain = editingDomain;
		this.fromFile = fromFile;
	}
	
	public Object getNewObject() {
		return editingDomain.load(fromFile).getContents().get(0);
	}

	public Object getObjectType() {
		return getMapping().get(fromFile.getFileExtension());
	}

	private Map<String, Object> getMapping() {
		if(mapping == null){
			mapping = new HashMap<String, Object>();
			mapping.put("pml", AlicaPackage.eINSTANCE.getPlan());
			mapping.put("beh", AlicaPackage.eINSTANCE.getBehaviour());
			mapping.put("pty", AlicaPackage.eINSTANCE.getPlanType());
		}
		return mapping;
	}

}
