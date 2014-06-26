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

import de.uni_kassel.vs.cn.planDesigner.alica.Task;


public class GraphDelta {
	
	/**
	 * Type indicating that the task is new. 
	 * 
	 */
	public static final int TYPE_NEW = 1;
	
	/**
	 * Type indicating that the task was already there but
	 * added another task with the same name. 
	 */
	public static final int TYPE_ADD = 2;
	
	/**
	 * Type indicating that the task was removed.
	 */
	public static final int TYPE_REMOVE = 3;
	
	/**
	 * Type indicating that the task was removed.
	 */
	public static final int TYPE_EDGE = 4;
	
	private Task affectedTask;
	
	private int type;
	
	private long existingTaskID;
	
	public GraphDelta(int type){
		this(null, type);
	}
	
	public GraphDelta(Task affectedTask, int type){
		this.affectedTask = affectedTask;
		this.type = type;
	}

	public Task getAffectedTask() {
		return affectedTask;
	}

	public void setAffectedTask(Task affectedTask) {
		this.affectedTask = affectedTask;
	}

	public int getType() {
		return type;
	}

	public void setType(int type) {
		this.type = type;
	}
	
	@Override
	public String toString() {
		String result = "GraphDelta (affectedTask: " +affectedTask +", type: ";
		switch(type){
		case TYPE_ADD:
			result += "ADD)";
			break;
		case TYPE_NEW:
			result += "NEW)";
			break;
		case TYPE_REMOVE:
			result += "REMOVE)";
			break;
		case TYPE_EDGE:
			result += "EDGE)";
			break;
		default:
			result += " UNKNOWN!!!)";
			break;
		}
		return result;
	}

	public long getExistingTaskID() {
		return existingTaskID;
	}

	public void setExistingTaskID(long existingTaskID) {
		this.existingTaskID = existingTaskID;
	}

}
