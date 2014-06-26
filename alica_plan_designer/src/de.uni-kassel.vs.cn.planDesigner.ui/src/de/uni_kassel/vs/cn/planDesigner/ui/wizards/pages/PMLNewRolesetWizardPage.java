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
package de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages;

import org.eclipse.core.resources.IContainer;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.Path;
import org.eclipse.jface.dialogs.IDialogPage;
import org.eclipse.jface.dialogs.IMessageProvider;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.ModifyEvent;
import org.eclipse.swt.events.ModifyListener;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;
import org.eclipse.ui.ISharedImages;
import org.eclipse.ui.PlatformUI;
import org.eclipse.ui.dialogs.ContainerSelectionDialog;

import de.uni_kassel.vs.cn.planDesigner.ui.util.CommonUtils;

/**
 * The "New" wizard page allows setting the container for the new file as well
 * as the file name. The page will only accept file name without the extension
 * OR with the extension that matches the expected one (pml).
 */

public class PMLNewRolesetWizardPage extends WizardPage {
	private Text containerText;

	private Text fileText;

	private ISelection selection;

	public PMLNewRolesetWizardPage(ISelection selection) {
		super("wizardPage");
		setTitle("New .rset File");
		setDescription("This wizard creates a new file with *.rset extension that can be opened by the Plan Designer.");
		this.selection = selection;
	}

	/**
	 * @see IDialogPage#createControl(Composite)
	 */
	public void createControl(Composite parent) {
		ISharedImages images = PlatformUI.getWorkbench().getSharedImages();
		 
		Composite container = new Composite(parent, SWT.NONE);
		GridLayout gLayout = new GridLayout(4,false);
		gLayout.verticalSpacing = 15;
		container.setLayout(gLayout);
		
		Label label = new Label(container, SWT.NONE);
		label.setText("&Container:");
		
		containerText = new Text(container, SWT.BORDER | SWT.SINGLE);
		containerText.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				dialogChanged();
			}
		});
		containerText.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,false));
		
		Button browseButton = new Button(container, SWT.PUSH);
		browseButton.setText("Browse...");
		browseButton.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(SelectionEvent e) {
				handleBrowse();
			}
		});

		Button defaultButton = new Button(container, SWT.PUSH);
		defaultButton.setText("Default");
		defaultButton.addSelectionListener(new SelectionAdapter() {
			public void widgetSelected(SelectionEvent e) {
				handleDefault();
			}
		});

		label = new Label(container, SWT.NONE);
		label.setImage(images.getImage(ISharedImages.IMG_OBJS_WARN_TSK));
		label.setLayoutData(new GridData(SWT.RIGHT,SWT.TOP,false,false));
		
		label = new Label(container, SWT.WRAP);
		label.setText("Note: It is recommended to save the Roleset in the same " +
				"directory as your roledefinition file to keep cross " +
				"reference URIs as short as possible. To choose the " +
				"recommended path, click \"Default\".");
		
		GridData gData = new GridData(SWT.FILL,SWT.FILL,true,false);
		gData.horizontalSpan = 3;
		gData.widthHint = 200;
		label.setLayoutData(gData);
		
		label = new Label(container, SWT.NONE);
		label.setText("&File name:");

		fileText = new Text(container, SWT.BORDER | SWT.SINGLE);
		fileText.addModifyListener(new ModifyListener() {
			public void modifyText(ModifyEvent e) {
				dialogChanged();
			}
		});
		gData = new GridData(SWT.FILL,SWT.FILL,true,false);
		gData.horizontalSpan = 3;
		fileText.setLayoutData(gData);
		
		initialize();
		dialogChanged();
		
		setControl(container);
		
	}
	
	

	protected void handleDefault() {
		containerText.setText(CommonUtils.getRoleDefinitionPath().removeLastSegments(1).toString());
	}

	/**
	 * Tests if the current workbench selection is a suitable container to use.
	 */

	private void initialize() {
		if (selection != null && selection.isEmpty() == false
				&& selection instanceof IStructuredSelection) {
			IStructuredSelection ssel = (IStructuredSelection) selection;
			if (ssel.size() > 1)
				return;
			Object obj = ssel.getFirstElement();
			if (obj instanceof IResource) {
				IContainer container;
				if (obj instanceof IContainer)
					container = (IContainer) obj;
				else
					container = ((IResource) obj).getParent();
				containerText.setText(container.getFullPath().toString());
			}
		}
		fileText.setText("Roleset.rset");
	}

	/**
	 * Uses the standard container selection dialog to choose the new value for
	 * the container field.
	 */

	private void handleBrowse() {
		ContainerSelectionDialog dialog = new ContainerSelectionDialog(
				getShell(), ResourcesPlugin.getWorkspace().getRoot(), false,
				"Select new file container");
		if (dialog.open() == ContainerSelectionDialog.OK) {
			Object[] result = dialog.getResult();
			if (result.length == 1) {
				containerText.setText(((Path) result[0]).toString());
			}
		}
	}

	private void dialogChanged() {
		IResource container = ResourcesPlugin.getWorkspace().getRoot()
				.findMember(new Path(getContainerName()));
		
		IResource recommendedContainer = getRecommendedContainer();
		
		String fileName = getFileName();

		if (getContainerName().length() == 0) {
			updateStatus("File container must be specified");
			return;
		}
		if (container == null
				|| (container.getType() & (IResource.PROJECT | IResource.FOLDER)) == 0) {
			updateStatus("File container must exist");
			return;
		}
		if (!container.isAccessible()) {
			updateStatus("Project must be writable");
			return;
		}
		if (fileName.length() == 0) {
			updateStatus("File name must be specified");
			return;
		}
		if (fileName.replace('\\', '/').indexOf('/', 1) > 0) {
			updateStatus("File name must be valid");
			return;
		}
		if (fileName.contains(" ")) {
			updateStatus("File name must not contain whitespaces");
			return;
		}
		int dotLoc = fileName.lastIndexOf('.');
		if (dotLoc != -1) {
			String ext = fileName.substring(dotLoc + 1);
			if (ext.equalsIgnoreCase("rset") == false) {
				updateStatus("File extension must be \"rset\"");
				return;
			}
		}
		if(CommonUtils.workspaceContainsFileName(CommonUtils.removeFileExtension(fileName)+".rset")){
			updateStatus("There is already a roleset with the same filename");
			return;
		}
		if(!container.equals(recommendedContainer)){
			updateStatus("The choosen container is not the recommended container!",IMessageProvider.WARNING);
			return;
		}
			
			
		updateStatus(null);
	}

	private IContainer getRecommendedContainer() {
		IContainer recommendedContainer = ResourcesPlugin.getWorkspace().getRoot()
						.findMember(CommonUtils.getRoleDefinitionPath()).getParent();
		return recommendedContainer;
	}
	
	private void updateStatus(String message){
		updateStatus(message, IMessageProvider.ERROR);
	}

	private void updateStatus(String message, int type) {
		setMessage(message, type);
		
		setPageComplete(message == null || type != IMessageProvider.ERROR);
	}

	public String getContainerName() {
		return containerText.getText();
	}

	public String getFileName() {
		return fileText.getText();
	}
}