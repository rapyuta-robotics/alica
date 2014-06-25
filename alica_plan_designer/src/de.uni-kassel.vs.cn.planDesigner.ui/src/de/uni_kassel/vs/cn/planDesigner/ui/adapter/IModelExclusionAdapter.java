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

import java.util.Set;

/**
 * This interface if implemented by clients who want to add a set
 * of EObjects which cannot be added to another EObject, even
 * if this object would be able to hold that particular object.
 * 
 * This is usefull to get a better control which model objects the user
 * can add to others e.g. with the palette
 * @author Zenobios
 *
 */
public interface IModelExclusionAdapter {

	Set<String> getExclusionClasses(); 
}
