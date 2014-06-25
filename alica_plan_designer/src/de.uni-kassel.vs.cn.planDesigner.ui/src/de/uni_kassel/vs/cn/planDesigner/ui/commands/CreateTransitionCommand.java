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

import java.util.Collections;

import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.TransactionalEditingDomain;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.Transition;

public class CreateTransitionCommand extends RecordingCommand {
	
	private TransactionalEditingDomain myDomain;
	
	private EObject parent;
	
	private State inState;
	private State outState;
	
	private Transition transition;

	public CreateTransitionCommand(TransactionalEditingDomain domain, Transition transition, EObject parent) {
		super(domain);
		this.myDomain = domain;
		this.transition = transition;
		this.parent = parent;
	}

	@Override
	protected void doExecute() {
		CompoundCommand compound = new CompoundCommand(0);
		
		compound.append(CreateChildCommand.create(myDomain, parent, new CommandParameter(null,null,transition), Collections.EMPTY_LIST));
		compound.append(SetCommand.create(myDomain, transition, AlicaPackage.eINSTANCE.getTransition_InState(), inState));
		compound.append(SetCommand.create(myDomain, transition, AlicaPackage.eINSTANCE.getTransition_OutState(), outState));

	}

	@Override
	public boolean canExecute() {
		return super.canExecute() && inState != null && outState != null;
	}

	public void setInPoint(State inState) {
		this.inState = inState;
	}

	public void setOutPoint(State outState) {
		this.outState = outState;
	}

}
