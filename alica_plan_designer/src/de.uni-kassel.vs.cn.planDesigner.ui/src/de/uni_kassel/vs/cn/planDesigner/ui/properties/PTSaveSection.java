package de.uni_kassel.vs.cn.planDesigner.ui.properties;


import java.io.IOException;

import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.CLabel;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.ui.views.properties.tabbed.TabbedPropertySheetPage;

import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;





public class PTSaveSection extends PMLPropertySection {

	private Button saveButton;

	@Override
	public void createControls(Composite parent,
			TabbedPropertySheetPage tabbedPropertySheetPage) {

		super.createControls(parent, tabbedPropertySheetPage);

		Composite composite = getWidgetFactory()
				.createFlatFormComposite(parent);
		
		CLabel l = getWidgetFactory().createCLabel(composite, "Note: Be aware that any changes to the plan type will affect all its occurences!");

		saveButton = getWidgetFactory().createButton(composite, "Save", SWT.PUSH);
		


		
		// Do some layout things
		FormData data = new FormData();
		data.left = new FormAttachment(0,0);
		l.setLayoutData(data);
		
		data = new FormData();
		data.left = new FormAttachment(0,0);
		data.top = new FormAttachment(l);
		saveButton.setLayoutData(data);
		
		registerListeners();
	}
	
	private void registerListeners(){
		getSaveButton().addListener(SWT.Selection, getEditController());		
	}
	
	@Override
	protected void selectionEvent(Object source) {
		System.out.println("Saving....");
		if(source.equals(getSaveButton())){
			getEditingDomain().getCommandStack().execute(new AbstractCommand(){

				@Override
				public boolean canUndo() {
					return false;
				}

				@Override
				protected boolean prepare() {
					return true;
				}

				public void execute() {
					PlanType pType = ((PlanType)PTSaveSection.this.getModel());
					
					pType.ensureParametrisationConsistency();
					
					// Save the planType
					Resource ptypeResource = pType.eResource();
					try {
						ptypeResource.save(ptypeResource.getResourceSet().getLoadOptions());
					} catch (IOException e) {
						e.printStackTrace();
					}
				}

				public void redo() {
					// Nothing to redo since we cannot undo
				}
				
			});
			
		}
	}
	
	@Override
	protected void basicSetInput(Object input) {
		super.basicSetInput(input);
		
		refreshControls();
	}
	
	/**
	 * Refreshes controls like buttons, doesn't refresh any viewers. It's intended to be used 
	 * from within refresh() to update the editable state of this section
	 */
	private void refreshControls(){
		// Enable the remove button if there are utilities
		getSaveButton().setEnabled(true);
		
	}
			
	
	@Override
	public boolean shouldUseExtraSpace() {
		return true;
	}

	protected Button getSaveButton() {
		return saveButton;
	}

	@Override
	public void refresh() {
		super.refresh();
		refreshControls();
	}
	@Override
	protected void updateView(Notification n) {	
	}


}
