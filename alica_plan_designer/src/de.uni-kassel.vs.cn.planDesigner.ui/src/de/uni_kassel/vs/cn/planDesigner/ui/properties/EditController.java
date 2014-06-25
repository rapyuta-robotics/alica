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

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.jface.viewers.ISelectionChangedListener;
import org.eclipse.jface.viewers.SelectionChangedEvent;
import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Widget;

import de.uni_kassel.vs.cn.planDesigner.ui.adapter.MultiObjectNotificationAdapter;

/**
 * The EditController acts as a controller which keeps multiple model objects in sync
 * with multiple views. The controller is able to pause listening to those views where
 * it has been registered to. While the controller is in pause-mode changes to the UI can
 * be made programatically without receiving SWT Events.
 * 
 * The EditController can be registered to receive model change notifications by calling 
 * addToObject(EObject). On the view side this controller can be registered as a SWT Listener
 * to receive arbitrary SWT Events.
 * @author Zenobios
 *
 */
public abstract class EditController extends MultiObjectNotificationAdapter implements Listener, ISelectionChangedListener{
	
	private boolean isPause;

	@Override
	public void doNotify(Notification n) {
		// Only forward the notification if the value changed
		if(!n.isTouch())
			handleNotification(n);
	}
	
	/**
	 * The model has changed, so tell the view that it should update itself. 
	 * @param n
	 */
	protected void handleNotification(Notification n){
		updateView(n);
	}

	public void handleEvent(Event event) {
		// Only react to SWT Events, if the controller is not in pause-mode
		if(!isPause){
			switch(event.type){
			case SWT.KeyDown:
				if(event.character == SWT.CR)
					enterPressed(event.widget);
				break;
			case SWT.FocusOut:
				focusOutEvent(event.widget);
				break;
			case SWT.Selection:
				selectionEvent(event.widget);
				break;
			case SWT.Modify:
				modifyEvent(event.widget);
				break;
			}
			
		}
	}
	
	public void selectionChanged(SelectionChangedEvent event) {
		selectionEvent(event.getSource());
	}
	
	/**
	 * A modify event occured in the given widget.
	 * @param source
	 */
	protected abstract void enterPressed(Widget source);
	
	/**
	 * A modify event occured in the given widget.
	 * @param source
	 */
	protected abstract void modifyEvent(Widget source);
	
	/**
	 * A selection event occured in the given object.
	 * @param source
	 */
	protected abstract void selectionEvent(Object source);
	
	/**
	 * The given widget has lost focus.
	 * @param source
	 */
	protected abstract void focusOutEvent(Widget source);
	
	/**
	 * Sets the controller into pause-mode. While the controller is in this mode, changes
	 * to those UI elements, for which this controller is registered to are no longer be
	 * processed until resumeListening() is called.
	 */
	public void pauseListening(){
		isPause = true;
	}
	
	/**
	 * Resumes listening to UI elements.
	 */
	public void resumeListening(){
		isPause = false;
	}

	/**
	 * Updates the view programatically, which means that the controller pauses
	 * listening to it's views, does to update and resumes listening. 
	 */
	public final void updateView(Notification n){
		pauseListening();
		doUpdateView(n);
		resumeListening();
	}
	
	public void dispose(){
		removeFromObjects();
	}
	
	/**
	 * Updates the view with information freshly obtained from the model(s). Note, 
	 * that the notification can be NULL in case of calling the updateView method
	 * directly! So NULL indicates that the view should update all information from the 
	 * model, because no information to update a particular element are available.
	 */
	protected abstract void doUpdateView(Notification n);
}
