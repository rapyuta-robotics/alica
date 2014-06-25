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
package de.uni_kassel.vs.cn.planDesigner.ui.adapter;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.emf.ecore.EAttribute;
import org.eclipse.emf.ecore.EClass;
import org.eclipse.emf.ecore.EDataType;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.ui.views.properties.ComboBoxPropertyDescriptor;
import org.eclipse.ui.views.properties.IPropertyDescriptor;
import org.eclipse.ui.views.properties.IPropertySource;
import org.eclipse.ui.views.properties.PropertyDescriptor;
import org.eclipse.ui.views.properties.TextPropertyDescriptor;

public class PropertySourceAdapter implements IPropertySource {
	
	/**
	 * The reference to the model for which this adapter is a adapter for.
	 */
	private EObject model;
	
	public PropertySourceAdapter(EObject model) {
		this.model = model;
	}

	public Object getEditableValue() {
		// TODO Auto-generated method stub
		return null;
	}

	public IPropertyDescriptor[] getPropertyDescriptors() {
		EClass cls = model.eClass();
		
		List<PropertyDescriptor> descriptors = new ArrayList<PropertyDescriptor>();
		System.out.println(model);
		for(EAttribute attr : cls.getEAllAttributes()){
			EDataType type = attr.getEAttributeType();
			System.out.println(attr.getName());
			if(attr.isID()){
				descriptors.add(new PropertyDescriptor(attr.getFeatureID(), attr.getName()));
			}else if(type.getInstanceClass() == boolean.class){
				descriptors.add(new ComboBoxPropertyDescriptor(attr.getFeatureID(), attr.getName(), new String[]{"true", "false"}));
			}else
				descriptors.add(new TextPropertyDescriptor(attr.getFeatureID(), attr.getName()));
		}
		
		return descriptors.toArray(new IPropertyDescriptor[descriptors.size()]);
	}

	public Object getPropertyValue(Object id) {
		EStructuralFeature feature = model.eClass().getEStructuralFeature(
				(Integer) id);
		System.out.println(model);
		Object result = null;
		if (feature != null) {
			Object val = model.eGet(feature);
			if (val == null) {
				// Ok, this is a bit tricky: the values of the attributes may not have
				// been initialized yet. If the datatype of an attribute is a boolean, we
				// have to return the appropriate Integer value (0 for false and 1 for true)
				// for the ComboBoxPropertyDescriptor. In all other cases, we return an empty
				// String
				if(feature.getEType().getInstanceClass() == boolean.class)
					result = new Integer(0);
				else
					result = "";
			} else if (val instanceof Boolean) {
				// The value is != null, so check if the type is a boolean and return the correct
				// Integer value
				result = ((Boolean) model.eGet(feature)).booleanValue() ? new Integer(
						0)
						: new Integer(1);
			} else
				result = model.eGet(feature).toString();
	
		}
		return result;
	}

	public boolean isPropertySet(Object id) {
		// TODO Auto-generated method stub
		return false;
	}

	public void resetPropertyValue(Object id) {
		// TODO Auto-generated method stub
		
	}

	public void setPropertyValue(Object id, Object value) {
		EStructuralFeature f = model.eClass().getEStructuralFeature((Integer)id);
		
		if(f != null){
			// Do some transformations depending on the type of the feature. Only a boolean
			// value is represented as an integer value (0 means True, 1 means false). 
			// All other values are String that have to be converted back to it's correct 
			// datatypes
			Class instanceClass = f.getEType().getInstanceClass();
			if(instanceClass == double.class){
				model.eSet(f, Double.parseDouble((String)value));
			}else if(instanceClass == float.class){
				model.eSet(f, Float.parseFloat((String)value));
			}else if(instanceClass == boolean.class){
				model.eSet(f, ((Integer)value).intValue() == 0 ? true : false);
			}else if(instanceClass == int.class){
				model.eSet(f, Integer.parseInt(((String)value)));
			}else
				model.eSet(f, value);
		}
	}

}
