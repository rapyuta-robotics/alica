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

import org.eclipse.emf.common.notify.Adapter;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.common.notify.Notifier;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.swt.widgets.Display;

public abstract class MultiObjectNotificationAdapter implements Adapter {
	
	
	static Notifier [] EMPTY = {};
	
	/**
	 * This list contains all EMF notifieres to which this adapter has
	 * been added to, i.e. to which objects, this adapter is listening
	 * for changes.
	 */
	List<Notifier> notifiers = new ArrayList<Notifier>();
	
	/**
	 * Adds the given EObject to the list of notifiers. From now on,
	 * this adapter starts listening for model changes from this 
	 * object.
	 * @param o
	 */
	public void addToObject(EObject o){
		// Do nothing if somebody trys to register this adapter 
		// more than one time to the object
		if(notifiers.contains(o))
			return;
		
		o.eAdapters().add(this);
		notifiers.add(o);
	}
	
	public abstract void doNotify(Notification n);
	
	public final void notifyChanged(final Notification n) {
		if (n.getEventType() == Notification.REMOVING_ADAPTER) {
			Object notifier = n.getNotifier();
			notifiers.remove(notifier);
		} else {
			Display current = Display.getCurrent();
			if(current == null)
			{
				current = Display.getDefault();
			}
			
			current.syncExec(new Runnable()
			{
				public void run()
				{
					doNotify(n);
				}
			});
		}
		
	}
	
	/**
	 * Removes this adapter from all objects it has been added to.
	 */
	public void removeFromObjects(){
		for(Notifier n : notifiers.toArray(EMPTY))
			n.eAdapters().remove(this);
	}
	
	/**
	 * Removes this adapter from the given object.
	 */
	public void removeFromObject(EObject o){
		o.eAdapters().remove(this);
	}

	public Notifier getTarget() {
		// TODO Auto-generated method stub
		return null;
	}

	public boolean isAdapterForType(Object type) {
		// TODO Auto-generated method stub
		return false;
	}

	public void setTarget(Notifier newTarget) {
		// TODO Auto-generated method stub

	}

}
