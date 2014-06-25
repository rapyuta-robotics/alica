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

import org.eclipse.core.runtime.IStatus;
import org.eclipse.core.runtime.Status;
import org.eclipse.emf.common.command.Command;
import org.eclipse.emf.common.command.CommandStack;
import org.eclipse.emf.common.notify.AdapterFactory;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.swt.events.DisposeEvent;
import org.eclipse.swt.events.DisposeListener;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Widget;
import org.eclipse.ui.IWorkbenchPart;
import org.eclipse.ui.views.properties.tabbed.AbstractPropertySection;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;

public abstract class PMLPropertySection extends AbstractPropertySection {
	
	private EObject model;
	
	private EditController editController;
	
	private PMLTabbedPropertySheetPage propertySheetPage;
	
	/**
	 * Checks if the input is an EObject and saves the reference if it is.
	 * @param input
	 */
	protected void basicSetInput(Object input){
		if(input instanceof EObject){
			this.model = (EObject)input;
		}
	}
	
	@Override
	public void setInput(IWorkbenchPart part, ISelection selection) {
		super.setInput(part, selection);
		
		if (!(selection instanceof IStructuredSelection)) {
			return ;
		}
		basicSetInput(((IStructuredSelection)selection).getFirstElement());
		
		addAllAdapters();
		
	}
	
	@Override
	public void createControls(Composite parent,
			TabbedPropertySheetPage tabbedPropertySheetPage) {
		super.createControls(parent, tabbedPropertySheetPage);
		
		// Save the reference to the tabbedPropertySheetPage to get access to the editor/commandFramework
		setPropertySheetPage((PMLTabbedPropertySheetPage)tabbedPropertySheetPage);
		
		parent.addDisposeListener(new DisposeListener() {
			public void widgetDisposed(DisposeEvent e) {
				dispose();
			}
		});
	}
	
	@Override
	public void dispose() {
		getEditController().dispose();
	}
	
	protected void addAllAdapters(){
		getEditController().addToObject(getModel());
	}
	
	protected void removeAllAdapters(){
		getEditController().removeFromObjects();
	}
	
	/**
	 * Convenience method which first removes all adapters and then 
	 * adds them again
	 */
	protected void refreshAdapters(){
		removeAllAdapters();
		addAllAdapters();
	}

	public EObject getModel() {
		System.out.println(model);
		return this.model;
	}
	
	/**
	 * Get's the EditController, creates one if first time called.
	 * @return
	 */
	public EditController getEditController() {
		if(editController == null){
			editController = new EditController(){
				// Just delegate all update methods from the controller to
				// methods from the sections.
				@Override
				protected void doUpdateView(Notification n) {
					PMLPropertySection.this.updateView(n);
				}

				@Override
				protected void focusOutEvent(Widget source) {
					PMLPropertySection.this.focusOutEvent(source);
				}

				@Override
				protected void modifyEvent(Widget source) {
					PMLPropertySection.this.modifyEvent(source);					
				}
				
				@Override
				protected void selectionEvent(Object source) {
					PMLPropertySection.this.selectionEvent(source);
				}

				@Override
				protected void enterPressed(Widget source) {
					PMLPropertySection.this.enterPressed(source);					
				}
			
			};
		}
		return editController;
	}
	
	/**
	 * Executes the given command via the editors commandstack.
	 * @param cmd
	 */
	protected void executeCommand(Command cmd){
		getCommandStack().execute(cmd);
	}
	
	public CommandStack getCommandStack(){
		return getPropertySheetPage().getCommandStackContributor().getEMFCommandStack();
	}
	
	public PMLTransactionalEditingDomain getEditingDomain(){
		return (PMLTransactionalEditingDomain) getPropertySheetPage().getCommandStackContributor().getEditingDomain();
	}
	
	protected AdapterFactory getAdapterFactory(){
		return getPropertySheetPage().getCommandStackContributor().getAdapterFactory();
	}

	private PMLTabbedPropertySheetPage getPropertySheetPage() {
		return propertySheetPage;
	}

	protected void setPropertySheetPage(PMLTabbedPropertySheetPage propertySheetPage) {
		this.propertySheetPage = propertySheetPage;
	}

	/**
	 * Updates the view with information freshly obtained from the model(s). This method
	 * is called if the controller receives model change notifications, so clients should 
	 * implement this method to keep the view in sync with the model.
	 */
	protected abstract void updateView(Notification n);
	
	/**
	 * Delivers a foucusOutEvent from the given Widget. Clients may implement this method 
	 * to create commands which modify the model accordingly.
	 * @param source
	 */
	protected void focusOutEvent(Widget source){
		// Does nothing by default
	}
	
	/**
	 * Delivers a modifiyEvent from the given Widget. Clients may implement this method 
	 * to create commands which modify the model accordingly.
	 * @param source
	 */
	protected void modifyEvent(Widget source){
		// Does nothing by default
	}
	
	/**
	 * Delivers a selectionEvent from the given Widget. Clients may implement this method 
	 * to create commands which modify the model accordingly.
	 * @param source
	 */
	protected void selectionEvent(Object source){
		// Does nothing by default
	}
	
	/**
	 * Delivers a enterPressed from the given Widget. Clients may implement this method 
	 * to create commands which modify the model accordingly.
	 * @param source
	 */
	protected void enterPressed(Widget source){
		// Does nothing by default
	}
	
	protected boolean isEditable(){
		boolean editable = false;
		
		// We check here if the model is contained in a resource to. Only
		// if we know the resource, we can determine if the resource is readonly.
		Resource r = getModel().eResource();
		if(r != null)
		{
			// FIXME: It isn't really clear why we sometimes get a NPE here.
			try
			{
				editable = !getEditingDomain().getResourceToReadOnlyMap().get(r);
			}
			catch(NullPointerException e)
			{
				StringBuffer msg = new StringBuffer();
				msg.append("Catched NPE in isEditable(): " +e +"\n");
				msg.append("\tEditingDomain: " +getEditingDomain() +"\n");
				if(getEditingDomain() != null)
				{
					msg.append("\tResourceToReadOnlyMap: " +getEditingDomain().getResourceToReadOnlyMap() +"\n");
				}
				PlanDesignerActivator.getDefault().getLog().log(
						new Status(IStatus.WARNING,PlanDesignerActivator.PLUGIN_ID,msg.toString(),e));
			}
		}

		return editable;
	}
}
