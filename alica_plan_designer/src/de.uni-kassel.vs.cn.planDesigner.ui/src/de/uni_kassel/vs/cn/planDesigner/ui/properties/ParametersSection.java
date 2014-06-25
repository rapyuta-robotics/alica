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

import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.RunnableWithResult;
import org.eclipse.jface.viewers.CellEditor;
import org.eclipse.jface.viewers.ColumnLabelProvider;
import org.eclipse.jface.viewers.ColumnViewer;
import org.eclipse.jface.viewers.EditingSupport;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.TableViewerColumn;
import org.eclipse.jface.viewers.TextCellEditor;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.viewers.ViewerSorter;
import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.CLabel;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;

public class ParametersSection extends PMLPropertySection {
	
	private class TableContentProvider implements IStructuredContentProvider{

		public Object[] getElements(final Object inputElement) {
			Object[] result = null	;
			try {
				result = (Object[])getEditingDomain().runExclusive(new RunnableWithResult.Impl<Object[]>(){
					public void run() {
						setResult(((BehaviourConfiguration)inputElement).getParameters().keySet().toArray());
					}
				});
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return result;
		}

		public void dispose() {
			getEditController().removeFromObjects();
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
//			// Remove the editController from the oldInput
//			if(oldInput != null)
//				getEditController().removeFromObject((EObject)oldInput);
//			
//			// And register it with the newInput
//			if(newInput != null)
//				getEditController().addToObject((EObject)newInput);
			
		}
		
	}
	
	private class ParameterEditingSupport extends EditingSupport{
		
		private boolean keyEditing;
		
		public ParameterEditingSupport(ColumnViewer viewer, boolean keyEditing) {
			super(viewer);
			this.keyEditing = keyEditing;
		}

		@Override
		protected boolean canEdit(Object element) {
			return ParametersSection.this.isEditable();
		}

		@Override
		protected CellEditor getCellEditor(Object element) {
			return new TextCellEditor((Composite)getViewer().getControl());
		}

		@Override
		protected Object getValue(final Object element) {
			if(keyEditing)
				return element.toString();
			else{
				String result = null;
				try {
					result = (String)getEditingDomain().runExclusive(new RunnableWithResult.Impl<String>(){
						public void run() {
							setResult(((BehaviourConfiguration)getModel()).getParameters().get(element));
						}
					});
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				return result;
			}
		}

		@Override
		protected void setValue(final Object element, final Object value) {
			CompoundCommand cmp = new CompoundCommand("Add Parameter");
			
			// Append a command that will update the view since the EMap doesn't fire
			// any modifications made to the model. So we wouldn't see the change
			cmp.append(new AbstractCommand(){
				// This command will actually modifiy the model
				RecordingCommand cmd = new RecordingCommand(getEditingDomain()){
					@Override
					protected void doExecute() {
						BehaviourConfiguration bConf = (BehaviourConfiguration)getModel();
						
						if(keyEditing){
							String oldValue = bConf.getParameters().get(element);
							System.out.println(oldValue);
							bConf.getParameters().remove(element);
							bConf.getParameters().put(value.toString(), oldValue);
						}else
							bConf.getParameters().put(element.toString(), value.toString());
					
					}
				};
				
				public void redo() {
					cmd.redo();
					if(!ParametersSection.this.getTViewer().getControl().isDisposed())
						updateView(null);
				}
				@Override
				public void undo() {
					cmd.undo();
					if(!ParametersSection.this.getTViewer().getControl().isDisposed())
						updateView(null);
				}
				public void execute() {
					cmd.execute();
					if(!ParametersSection.this.getTViewer().getControl().isDisposed())
						updateView(null);
				}
				
				@Override
				public boolean canExecute() {
					return cmd.canExecute();
				}
				
				@Override
				public boolean canUndo() {
					return cmd.canUndo();
				}
				
			});
			
			executeCommand(cmp);
		}
		
	}

	private Button addButton;

	private Button removeButton;

	private TableViewer tViewer;

	
	@Override
	public void createControls(Composite parent,
			TabbedPropertySheetPage tabbedPropertySheetPage) {

		super.createControls(parent, tabbedPropertySheetPage);

		Composite composite = getWidgetFactory()
				.createFlatFormComposite(parent);
		
		CLabel l = getWidgetFactory().createCLabel(composite, "Note: Be aware that changing the parameters of this configuration means that in every " +
				"state this configuration is used, it will have the same parameters since it is the same configuration!");

		addButton = getWidgetFactory().createButton(composite, "Add", SWT.PUSH);
		removeButton = getWidgetFactory().createButton(composite, "Remove",	SWT.PUSH);

		tViewer = new TableViewer(composite, SWT.BORDER | SWT.FULL_SELECTION);
		tViewer.setColumnProperties(new String[]{"Name", "Value"});
		tViewer.getTable().setLinesVisible(true);
		tViewer.getTable().setHeaderVisible(true);


		TableViewerColumn column = new TableViewerColumn(tViewer, SWT.NONE);
		column.getColumn().setWidth(200);
		column.getColumn().setText("Name");
		column.setLabelProvider(new ColumnLabelProvider() {
			public String getText(Object element) {
				return element.toString();
			}
		});
		column.setEditingSupport(new ParameterEditingSupport(tViewer,true));
		
		column = new TableViewerColumn(tViewer, SWT.NONE);
		column.getColumn().setWidth(200);
		column.getColumn().setText("Value");
		column.setLabelProvider(new ColumnLabelProvider() {
			public String getText(final Object element) {
				String result = null;
				try {
					result = (String)getEditingDomain().runExclusive(new RunnableWithResult.Impl<String>(){
						public void run() {
							setResult(((BehaviourConfiguration)getModel()).getParameters().get(element));
						}
					});
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				return result;
			}
		});
		column.setEditingSupport(new ParameterEditingSupport(tViewer,false));
		
		// Do some layout things
		FormData data = new FormData();
		data.left = new FormAttachment(0,0);
		l.setLayoutData(data);
		
		data = new FormData();
		data.left = new FormAttachment(0,0);
		data.top = new FormAttachment(l);
		addButton.setLayoutData(data);
		
		data = new FormData();
		data.left = new FormAttachment(addButton);
		data.top = new FormAttachment(l);
		removeButton.setLayoutData(data);

		data = new FormData();
		data.top = new FormAttachment(addButton,0, SWT.DEFAULT);
		data.bottom = new FormAttachment(100,0);
		data.right = new FormAttachment(100,0);
		data.left = new FormAttachment(0,0);
		tViewer.getControl().setLayoutData(data);
		
		tViewer.setContentProvider(new TableContentProvider());
		tViewer.setSorter(new ViewerSorter());
		registerListeners();
	}
	
	private void registerListeners(){
		getAddButton().addListener(SWT.Selection, getEditController());
		getRemoveButton().addListener(SWT.Selection, getEditController());
	}
	
	@Override
	protected void selectionEvent(Object source) {
		if(source.equals(getAddButton())){
			getRemoveButton().setEnabled(true);
			
			final String key = "NewParam";

			executeCommand(new RecordingCommand(getEditingDomain(), "Add Parameter"){
				@Override
				protected void doExecute() {
					((BehaviourConfiguration)getModel()).getParameters().put(key, "");
				}
			});
			
			getTViewer().setSelection(new StructuredSelection(key), true);
		}else if(source.equals(getRemoveButton())){
			final Object selection = ((IStructuredSelection)getTViewer().getSelection()).getFirstElement();
			int oldIdx = getTViewer().getTable().getSelectionIndex();
			if(selection != null){
				
				executeCommand(new RecordingCommand(getEditingDomain(), "Remove Parameter"){
					@Override
					protected void doExecute() {
						((BehaviourConfiguration)getModel()).getParameters().remove(selection);
					}
				});
			}
			
			if(!hasParameters())
				getRemoveButton().setEnabled(false);
			else{
				getTViewer().getTable().setSelection(oldIdx - 1 < 0 ? oldIdx : (oldIdx-1));
			}
			
		}
	}
	
	@Override
	protected void basicSetInput(Object input) {
		super.basicSetInput(input);
		getTViewer().setInput(getModel());
		
		refreshControls();
	}
	
	/**
	 * Refreshes controls like buttons, doesn't refresh any viewers. It's intended to be used 
	 * from within refresh() to update the editable state of this section
	 */
	private void refreshControls(){
		// Enable the remove button if there are utilities
		getRemoveButton().setEnabled(false);
		getAddButton().setEnabled(false);
		
		if(isEditable()){
			getAddButton().setEnabled(true);
			if(hasParameters())
				getRemoveButton().setEnabled(true);
		}
	}
	
	public TableViewer getTViewer() {
		return tViewer;
	}
	
	private boolean hasParameters(){
		boolean result = false;
		try {
			result = (Boolean)getEditingDomain().runExclusive(new RunnableWithResult.Impl<Boolean>(){
				public void run() {
					setResult(((BehaviourConfiguration)getModel()).getParameters().size() > 0);
				}
			});
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return result;
	}
	
	@Override
	protected void updateView(Notification n) {
		getTViewer().refresh();
	}
	
	@Override
	public boolean shouldUseExtraSpace() {
		return true;
	}

	protected Button getAddButton() {
		return addButton;
	}

	protected Button getRemoveButton() {
		return removeButton;
	}
	
	@Override
	public void refresh() {
		super.refresh();
		refreshControls();
	}

}
