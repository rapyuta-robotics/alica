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
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.jface.viewers.ILabelProviderListener;
import org.eclipse.jface.viewers.IStructuredContentProvider;
import org.eclipse.jface.viewers.ITableLabelProvider;
import org.eclipse.jface.viewers.TableViewer;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.swt.SWT;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.layout.RowLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Table;
import org.eclipse.swt.widgets.TableColumn;
import org.eclipse.swt.widgets.TableItem;
import org.eclipse.ui.forms.widgets.FormToolkit;
import org.eclipse.ui.forms.widgets.Section;

import de.uni_kassel.vs.cn.planDesigner.alica.Role;
import de.uni_kassel.vs.cn.planDesigner.alica.RoleSet;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUIExtensionModelPackage;
import de.uni_kassel.vs.cn.planDesigner.pmlextension.uiextensionmodel.PmlUiExtension;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.ExecuteAndRefreshGraphCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.ResizeToPreferredSizeCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.RolesetEditor;

public class ToolbarComposite extends Composite {
	
	private class RolesetTableContentProvider implements IStructuredContentProvider{

		public Object[] getElements(Object inputElement) {
			RoleSet roleset = (RoleSet)inputElement;
			Role[] roles = new Role[roleset.getMappings().size()];
			for(int i=0; i < roleset.getMappings().size(); i++){
				roles[i] = roleset.getMappings().get(i).getRole();
			}
			return roles;
		}

		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
		}
		
	}
	
	private class RolesetTableLabelProvider implements ITableLabelProvider{

		public Image getColumnImage(Object element, int columnIndex) {
			return null;
		}

		public String getColumnText(Object element, int columnIndex) {
			String result = "";
			switch(columnIndex){
			case 0:
				break;
			case 1:
				result = ((Role)element).getName();
				break;
			}
			return result;
		}

		public void addListener(ILabelProviderListener listener) {
			// TODO Auto-generated method stub
			
		}

		public void dispose() {
			// TODO Auto-generated method stub
			
		}

		public boolean isLabelProperty(Object element, String property) {
			// TODO Auto-generated method stub
			return false;
		}

		public void removeListener(ILabelProviderListener listener) {
			// TODO Auto-generated method stub
			
		}
		
	}

	private FormToolkit toolkit;
	private RoleSet roleset;
	private TableViewer roleFilterViewer;
	private RolesetEditor editor;

	public ToolbarComposite(RolesetEditor editor, Composite parent, int style) {
		super(parent, style);
		this.editor = editor;
		this.roleset = editor.getRoleSet();
		GridLayout gLayout = new GridLayout();
		gLayout.marginHeight = 0;
		gLayout.marginWidth = 0;
		gLayout.horizontalSpacing = 0;
		gLayout.verticalSpacing = 0;

		setLayout(gLayout);
		toolkit = new FormToolkit(parent.getDisplay());
		createToolbar();
	}

	private void createToolbar() {
		Section toolSection = toolkit.createSection(this, Section.TITLE_BAR | Section.TWISTIE);
		toolSection.setText("Toolbox");
		toolSection.setLayout(new FillLayout());
		toolSection.setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, false));

		Composite sectionClient = toolkit.createComposite(toolSection);
		sectionClient.setLayout(new RowLayout());
		toolSection.setClient(sectionClient);

		roleFilterViewer = new TableViewer(sectionClient, SWT.FULL_SELECTION | SWT.BORDER | SWT.CHECK);
		
		roleFilterViewer.setColumnProperties(new String[]{"Filter", "Role"});
		roleFilterViewer.setUseHashlookup(true);
		Table table = roleFilterViewer.getTable();
		table.setHeaderVisible(true);
		
		TableColumn col = new TableColumn(table,SWT.CENTER);
		col.setText("Filter");
		col.setWidth(50);
		
		col = new TableColumn(table, SWT.LEFT);
		col.setText("Role");
		col.setWidth(250);
		
		roleFilterViewer.setContentProvider(new RolesetTableContentProvider());
		roleFilterViewer.setLabelProvider(new RolesetTableLabelProvider());
		
		roleFilterViewer.setInput(roleset);
		
		initFiltering();
		
		table.addListener (SWT.Selection, new Listener () {
			public void handleEvent (Event event) {
				if(event.detail == SWT.CHECK){
					handleFiltering(event);
				}
			}
		});
	}

	private void initFiltering() {
		for(TableItem item : getRoleFilterViewer().getTable().getItems()){
			Role r = (Role)item.getData();
			item.setChecked(isRoleFiltered(r));
		}
		
	}

	protected void handleFiltering(Event event) {
		TableItem item = ((TableItem)event.item);
		Role r = (Role)item.getData();
		boolean filtered = item.getChecked();
		
		filterRole(r, filtered);
		
	}
	
	private boolean isRoleFiltered(Role r){
		return getUIExtension(r).isCollapsed();
	}
	
	private PmlUiExtension getUIExtension(Role r){
		return editor.getUIExtension(r, true);
	}
	
	private void filterRole(Role role, boolean filter){
		// Collapsing a role will be handled as hiding at that point
		Command filterCmd = SetCommand.create(
					editor.getEditingDomain(), 
					getUIExtension(role), 
					PmlUIExtensionModelPackage.eINSTANCE.getPmlUiExtension_Collapsed(), 
					filter);
		
		ExecuteAndRefreshGraphCommand cmd = new ExecuteAndRefreshGraphCommand(
				(filter ? "Filter " : "Show ") +role.getName(),filterCmd, editor.getGraphicalViewer(),editor.getTaskGraph());
		
		CompoundCommand cmp = new CompoundCommand(0);
		cmp.append(cmd);
		cmp.append(new ResizeToPreferredSizeCommand(editor));
		
		editor.getEMFCommandStack().execute(cmp);
//		System.out.println("Role: " +role + " is filtered: " +filter);
	}

	private TableViewer getRoleFilterViewer() {
		return roleFilterViewer;
	}

}
