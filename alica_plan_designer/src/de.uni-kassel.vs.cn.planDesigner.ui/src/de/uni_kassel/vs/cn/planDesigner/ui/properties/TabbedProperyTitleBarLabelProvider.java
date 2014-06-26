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
package de.uni_kassel.vs.cn.planDesigner.ui.properties;

import java.util.ArrayList;
import java.util.Collection;

import org.eclipse.emf.edit.provider.ComposedAdapterFactory;
import org.eclipse.emf.edit.provider.IItemLabelProvider;
import org.eclipse.emf.edit.ui.provider.AdapterFactoryLabelProvider;
import org.eclipse.jface.viewers.ILabelProvider;
import org.eclipse.jface.viewers.ILabelProviderListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.swt.graphics.Image;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;

public class TabbedProperyTitleBarLabelProvider implements ILabelProvider {
	
	private AdapterFactoryLabelProvider delegate;
	
	public Image getImage(Object element) {
		return getDelegate().getImage(calculateSelection(element));
	}

	public String getText(Object element) {
		return getDelegate().getText(calculateSelection(element));
	}

	public void addListener(ILabelProviderListener listener) {
		getDelegate().addListener(listener);
	}

	public void dispose() {
		getDelegate().dispose();
	}

	public boolean isLabelProperty(Object element, String property) {
		return getDelegate().isLabelProperty(calculateSelection(element), property);
	}

	public void removeListener(ILabelProviderListener listener) {
		getDelegate().removeListener(listener);
	}

	public AdapterFactoryLabelProvider getDelegate() {
		if(delegate == null){
			Collection<Object> types = new ArrayList<Object>();
			types.add(AlicaPackage.eINSTANCE);
			types.add(IItemLabelProvider.class);
			delegate = new AdapterFactoryLabelProvider(new ComposedAdapterFactory(ComposedAdapterFactory.Descriptor.Registry.INSTANCE).getFactoryForTypes(types));
		}
		
		
		return delegate;
	}
	
	private Object calculateSelection(Object element){
		if(element instanceof IStructuredSelection){
			return ((IStructuredSelection) element).getFirstElement();
		}else
			return element;
	}

}
