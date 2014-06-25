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

import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.transaction.NotificationFilter;
import org.eclipse.emf.transaction.ResourceSetChangeEvent;
import org.eclipse.emf.transaction.ResourceSetListener;
import org.eclipse.emf.transaction.RollbackException;

public class ResourceSetChangeListener implements ResourceSetListener {

	public NotificationFilter getFilter() {
		// TODO Auto-generated method stub
		return null;
	}

	public boolean isAggregatePrecommitListener() {
		// TODO Auto-generated method stub
		return false;
	}

	public boolean isPostcommitOnly() {
		// TODO Auto-generated method stub
		return false;
	}

	public boolean isPrecommitOnly() {
		// TODO Auto-generated method stub
		return false;
	}

	public void resourceSetChanged(ResourceSetChangeEvent event) {
//		System.out.println("\n------------------------------------------\n");
//		 System.out.println("Domain " + event.getEditingDomain().getID() +
//	             " changed " + event.getNotifications().size() + " times" );
//		 
//		 for(Notification n : event.getNotifications()){
//			 System.out.println("\tNotifaction -> \n" 
//					 +"\t\tType: " +n.getEventType() 
//					 +", \n\t\tFeatureID: " +n.getFeatureID(null) +", Feature: " +n.getFeature()
//					 +", \n\t\tNotifier: " +n.getNotifier()
//					 +", \n\t\tOldValue: " +n.getOldValue()
//					 +", \n\t\tNewValue: " +n.getNewValue());
//					 
//		 }
//		 System.out.println("\n------------------------------------------\n");
	}

	public Command transactionAboutToCommit(ResourceSetChangeEvent event)
			throws RollbackException {
		// TODO Auto-generated method stub
		return null;
	}

}
