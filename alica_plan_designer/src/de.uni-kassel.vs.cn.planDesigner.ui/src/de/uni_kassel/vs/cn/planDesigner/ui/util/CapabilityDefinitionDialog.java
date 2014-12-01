package de.uni_kassel.vs.cn.planDesigner.ui.util;

import java.util.Collections;
import java.util.List;

import org.eclipse.emf.common.notify.Notification;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.edit.command.CommandParameter;
import org.eclipse.emf.edit.command.CreateChildCommand;
import org.eclipse.emf.edit.command.RemoveCommand;
import org.eclipse.emf.edit.command.SetCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.jface.dialogs.Dialog;
import org.eclipse.jface.viewers.ILabelProvider;
import org.eclipse.jface.viewers.ILabelProviderListener;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.ITreeContentProvider;
import org.eclipse.jface.viewers.StructuredSelection;
import org.eclipse.jface.viewers.TreeViewer;
import org.eclipse.jface.viewers.Viewer;
import org.eclipse.jface.window.IShellProvider;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.DisposeEvent;
import org.eclipse.swt.events.DisposeListener;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.FormAttachment;
import org.eclipse.swt.layout.FormData;
import org.eclipse.swt.layout.FormLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Control;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Widget;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
//import de.uni_kassel.vs.cn.alica.Role;
//import de.uni_kassel.vs.cn.alica.RoleDefinitionSet;
import de.uni_kassel.vs.cn.planDesigner.alica.CapValue;
import de.uni_kassel.vs.cn.planDesigner.alica.Capability;
import de.uni_kassel.vs.cn.planDesigner.alica.CapabilityDefinitionSet;
//import de.uni_kassel.vs.cn.alica.Characteristic;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanElement;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.properties.EditController;

public class CapabilityDefinitionDialog extends Dialog {
	
	private class CapabilityLabelProvider implements ILabelProvider{

		public Image getImage(Object element) {
			if(element instanceof Capability)
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_CAPABILITY_16);
			else if(element instanceof CapValue)
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_CAPABILITY_16);
			else
				return PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_UNKNOWN_TYPE);
		}

		public String getText(Object element) {
			if(element instanceof PlanElement){
				return ((PlanElement)element).getName();
			}else
				return "Unknown element";
		}

		public void addListener(ILabelProviderListener listener) {
		}

		public void dispose() {
		}

		public boolean isLabelProperty(Object element, String property) {
			// TODO Optimize this
			return true;
		}

		public void removeListener(ILabelProviderListener listener) {
		}
		
	}
	
	private class CapabilityContentProvider implements ITreeContentProvider{

		public Object[] getChildren(Object parentElement) {
			if(parentElement instanceof Capability){
				return ((Capability)parentElement).getCapValues().toArray();
			}else
				return null;
		}

		public Object getParent(Object element) {
			if(element instanceof EObject)
				return ((EObject)element).eContainer();
			else
				return null;
		}

		public boolean hasChildren(Object element) {
			if(element instanceof Capability)
				return !((Capability)element).getCapValues().isEmpty();
			else
				return false;
		}

		public Object[] getElements(Object inputElement) {
			// The input is expected to be an CapabilityDefinitionSet
			if(inputElement instanceof CapabilityDefinitionSet){
				return ((CapabilityDefinitionSet)inputElement).getCapabilities().toArray();
			}
			return new Object[]{"Input invalid"};
		}

		public void dispose() {
		}

		public void inputChanged(Viewer viewer, Object oldInput, Object newInput) {
			
		}
		
	}
	
	private Group nothingToDisplayGroup;
	
	private Group capabilityDataGroup;
	
	private Group capValueDataGroup;
	
	private TreeViewer capabilityViewer;
	
	private Button addCapabilityButton;
	private Button removeButton;
	private Button addCapValueButton;
	
	private Text capNameText;
	private Text capValueText;
	
	//private Text keyText;
	//private Text valueText;
	//private Text weightText;
	
	private Composite container;
	
	private CapabilityDefinitionSet input;
	
	private Object currentSelection;
	
	private EditController editController;

	private PMLTransactionalEditingDomain domain;
	private Shell shell;
	
	public CapabilityDefinitionDialog(IShellProvider provider, PMLTransactionalEditingDomain domain) {
		super(provider);
		this.domain = domain;
	}
	
	@Override
	protected Control createDialogArea(Composite parent) {
		container = (Composite)super.createDialogArea(parent);

		// Layout for the composite
		GridLayout gLayout = new GridLayout(2,false);
		container.setLayout(gLayout);
		
		// Initialize the header
		initializeHeader(container);
		
		// Init the left side. This will always be the same
		Composite left = createLeftSide(container);
		GridData gData = new GridData(SWT.FILL,SWT.FILL,true,true);
		left.setLayoutData(gData);
		
		// Initially we will display the nothing to display composite
		displayComposite(getNothingToDisplayGroup());
		
		container.addDisposeListener(new DisposeListener(){
			public void widgetDisposed(DisposeEvent e) {
				getEditController().removeFromObjects();
			}
		});
		
		return container;
	}
	
	/**
	 * Sets the input of this dialog. The given Resources content is expected to be an
	 * instance of P
	 * @param res
	 */
	public void setInput(Resource res){
		this.input = (CapabilityDefinitionSet)res.getContents().get(0);
		
		registerController(input);
		
		// Apply the input to the capabilityViewer
		getCapabilityViewer().setInput(input);
		
		if(!input.getCapabilities().isEmpty()){
			// Select the first entry
			getCapabilityViewer().setSelection(new StructuredSelection(input.getCapabilities().get(0)));
		}
		
	}
	
	private void registerController(CapabilityDefinitionSet capabilityDef){
		EditController controller = getEditController();
		
		controller.addToObject(capabilityDef);
		
		List<Capability> capabilities =  capabilityDef.getCapabilities();
		for(Capability r : capabilities){
			controller.addToObject(r);
			for(CapValue c : r.getCapValues())
				controller.addToObject(c);
		}
	}
	
	/** 
	 * Displays the given composite and hides the others
	 * @param compToDisplay
	 */
	private void displayComposite(Composite compToDisplay){
		GridData gData = null;

		gData = (GridData)getCapabilityDataGroup().getLayoutData();
		gData.exclude = compToDisplay.equals(getCapabilityDataGroup()) ? false : true;
		
		gData = (GridData)getCapValueDataGroup().getLayoutData();
		gData.exclude = compToDisplay.equals(getCapValueDataGroup()) ? false : true;
		
		gData = (GridData)getNothingToDisplayGroup().getLayoutData();
		gData.exclude = compToDisplay.equals(getNothingToDisplayGroup()) ? false : true;
		
		compToDisplay.getParent().layout();
		
		// TODO: That could be done better!
		if(compToDisplay.equals(getCapValueDataGroup())){
			getNothingToDisplayGroup().setVisible(false);
			getCapabilityDataGroup().setVisible(false);
		}else if(compToDisplay.equals(getNothingToDisplayGroup())){
			getCapValueDataGroup().setVisible(false);
			getCapabilityDataGroup().setVisible(false);
		}else{
			getNothingToDisplayGroup().setVisible(false);
			getCapValueDataGroup().setVisible(false);
		}
		compToDisplay.setVisible(true);
	}
	
	/**
	 * Refrehes the UI with information freshly obtained from the model
	 */
	private void refreshCapabilityGroup(){
		Capability cap = (Capability)currentSelection;
		capNameText.setText(cap.getName());
	}
	
	/**
	 * Refrehes the UI with information freshly obtained from the model
	 */
	private void refreshCapValueGroup(){
		CapValue capVal = (CapValue)currentSelection;
		capValueText.setText(capVal.getName());
		//capValueText.setData(capVal.getName(), capVal);
	}
	
	private void initializeHeader(Composite parent){
		Composite descContainer = new Composite(parent, SWT.NONE);
		GridData gData = new GridData(SWT.FILL,SWT.FILL,true,false);
		gData.horizontalSpan = 2;
		descContainer.setLayoutData(gData);
		descContainer.setLayout(new FormLayout());
		
		Label imgLabel = new Label(descContainer,SWT.NONE);
		imgLabel.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_ROLE_DEFINITION_64));
		imgLabel.setLayoutData(new FormData());
		
		Label desc = new Label(descContainer, SWT.NONE);
		desc.setText("Define capabilities and their values. " +
				"\nNote: If you add/remove capabilities, ensure to adjust your roledefinition sets!");
		FormData fData = new FormData();
		fData.left = new FormAttachment(imgLabel, 20);
		desc.setLayoutData(fData);
	}
	
	private void intializeButtonsContainer(Composite parent){
		Composite buttonsContainer = new Composite(parent,SWT.NONE);
		buttonsContainer.setLayoutData(new GridData(SWT.CENTER,SWT.CENTER,false,false));
		FormLayout fLayout = new FormLayout();
		fLayout.spacing = 15;
		buttonsContainer.setLayout(fLayout);
		
		addCapabilityButton = new Button(buttonsContainer, SWT.PUSH);
		addCapabilityButton.setText("Add Capability");
		addCapabilityButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				Capability r = AlicaFactory.eINSTANCE.createCapability();
				// Register the editController as adapter
				getEditController().addToObject(r);
				
				domain.getCommandStack().execute(
						CreateChildCommand.create(
								domain, 
								input, 
								new CommandParameter(input,AlicaPackage.eINSTANCE.getCapabilityDefinitionSet_Capabilities(),r), 
								Collections.emptyList()));
				
				// Select the new Capability
			
				getCapabilityViewer().setSelection(new StructuredSelection(r),true);
				removeButton.setEnabled(true);
			}
		});
		
		removeButton = new Button(buttonsContainer, SWT.PUSH);
		removeButton.setText("Remove");
		removeButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				// Remove the editController from the object
				getEditController().removeFromObject((EObject)currentSelection);
				
				domain.getCommandStack().execute(
						RemoveCommand.create(
								domain, 
								Collections.singleton(currentSelection)));
				
				if(input.getCapabilities().isEmpty()){
					removeButton.setEnabled(false);
					getCapabilityViewer().setSelection(new StructuredSelection(StructuredSelection.EMPTY));
				} else{
					removeButton.setEnabled(true);
					
					getCapabilityViewer().setSelection(new StructuredSelection(input.getCapabilities().get(0)));
				}
					
			}
		});
		
		addCapValueButton = new Button(buttonsContainer, SWT.PUSH);
		addCapValueButton.setText("Add Cap. Value");
		addCapValueButton.addSelectionListener(new SelectionAdapter(){
			@Override
			public void widgetSelected(SelectionEvent e) {
				Capability r = (Capability)currentSelection;
				System.out.println("cap values b4: "+r.getCapValues().size());
				System.out.println(r);
				CapValue c = AlicaFactory.eINSTANCE.createCapValue();
				// Register the editController as adapter
				getEditController().addToObject(c);
				
				domain.getCommandStack().execute(
						CreateChildCommand.create(
								domain, 
								r, 
								//new CommandParameter(r,r.getCapValues(),c),
								new CommandParameter(r,AlicaPackage.eINSTANCE.getCapability_CapValues(),c), 
								Collections.emptyList()));
				
				// Select the new CapValue
				System.out.println("cap values after: "+r.getCapValues().size());
				getCapabilityViewer().setSelection(new StructuredSelection(c),true);
			}
		});
		addCapValueButton.setEnabled(false);
		
		// Layout for the buttonsContainer
		FormData fData = new FormData();
		fData.top = new FormAttachment();
		fData.width = 140;
		addCapabilityButton.setLayoutData(fData);
		
		fData = new FormData();
		fData.top = new FormAttachment(addCapabilityButton);
		fData.width = 140;
		addCapValueButton.setLayoutData(fData);
		
		fData = new FormData();
		fData.top = new FormAttachment(addCapValueButton,15);
		fData.width = 140;
		removeButton.setLayoutData(fData);
		
	}

	@Override
	protected void configureShell(Shell newShell) {
		super.configureShell(newShell);
		newShell.setText("Capability Definition");
		newShell.setImage(PlanDesignerActivator.getDefault().getImageRegistry().get(PlanDesignerConstants.ICON_CAPABILITY_DEFINITION_16));
		this.shell = newShell;
	}

	private Group getNothingToDisplayGroup() {
		if(nothingToDisplayGroup == null){
			nothingToDisplayGroup = createNothingToDisplayGroup(container);
		}
		return nothingToDisplayGroup;
	}

	private Group getCapabilityDataGroup() {
		if(capabilityDataGroup == null){
			capabilityDataGroup = createCapabilityDataGroup(container);
		}
		return capabilityDataGroup;
	}

	private Group getCapValueDataGroup() {
		if(capValueDataGroup == null){
			capValueDataGroup = createCapValueDataGroup(container);
		}
		return capValueDataGroup;
	}


	
	private Group createNothingToDisplayGroup(Composite parent) {
		Group ntdComposite = new Group(parent, SWT.NONE);
		ntdComposite.setText("No Information available");
		FillLayout fLayout = new FillLayout();
		fLayout.marginHeight = 15;
		fLayout.marginHeight = 15;
		ntdComposite.setLayout(fLayout);
		ntdComposite.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
		
		Label l = new Label(ntdComposite, SWT.WRAP);
		l.setText("Nothing to display. Create a Capability by clicking \"Add Capability\"");
		
		return ntdComposite;
	}
	
	private Group createCapabilityDataGroup(Composite parent) {
		Group rDataComp = new Group(parent, SWT.NONE);
		rDataComp.setText("Capability");
		rDataComp.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
		rDataComp.setLayout(PlanEditorUtils.createFormLayout());
		
		Label nameLabel = new Label(rDataComp, SWT.NONE);
		nameLabel.setText("Name:");
		
		capNameText = new Text(rDataComp, SWT.BORDER);
		capNameText.addListener(SWT.FocusIn, getEditController());
		capNameText.addListener(SWT.FocusOut, new Listener() {
		      public void handleEvent(Event e) {
		    	applyCapabilityData();
		      }
		});
		
		// Layout
		FormData fData = new FormData();
		fData.width = 40;
		nameLabel.setLayoutData(fData);
		
		fData = new FormData();
		fData.left = new FormAttachment(nameLabel);
		fData.width = 200;
		capNameText.setLayoutData(fData);
		
		return rDataComp;
	}

	private Group createCapValueDataGroup(Composite parent) {
		Group cDataComp = new Group(parent,SWT.NONE);
		cDataComp.setText("CapValue");
		cDataComp.setLayout(PlanEditorUtils.createFormLayout());
		cDataComp.setLayoutData(new GridData(SWT.FILL,SWT.FILL,true,true));
		
		Label valueLabel = new Label(cDataComp, SWT.NONE);
		valueLabel.setText("Value:");
		
		capValueText = new Text(cDataComp, SWT.BORDER);
		capValueText.addListener(SWT.FocusIn, getEditController());
		capValueText.addListener(SWT.FocusOut, new Listener() {
		      public void handleEvent(Event e) {
		    	  applyCapValuesData();
		      }
		});

		// Layout of CharNameLabel
		FormData fData = new FormData();
		fData.width = 50;
		valueLabel.setLayoutData(fData);
		
		// Layout of capValueText
		fData = new FormData();
		fData.left = new FormAttachment(valueLabel);
		fData.width = 200;
		capValueText.setLayoutData(fData);
		
					
		return cDataComp;
	}
	
	/**
	 * Creates the left side which will display the Capability tree and has the
	 * buttons to add/remove capabilities and CapValues
	 * @param parent
	 */
	private Composite createLeftSide(Composite parent) {
		Group left = new Group(parent, SWT.NONE);
		left.setText("capabilities and CapValues");
		left.setLayout(new GridLayout(2,false));
	
		// Add the capabilityViewer
		capabilityViewer = new TreeViewer(left);
		GridData gData = new GridData(SWT.FILL,SWT.FILL,true,true);
		gData.widthHint = 180;
		gData.heightHint = 300;
		capabilityViewer.getTree().setLayoutData(gData);
		capabilityViewer.setContentProvider(new CapabilityContentProvider());
		capabilityViewer.setLabelProvider(new CapabilityLabelProvider());
		
		capabilityViewer.addSelectionChangedListener(getEditController());
//		capabilityViewer.getTree().addListener(SWT.DefaultSelection, getEditController());
		
		// Add the buttons
		intializeButtonsContainer(left);
		
		return left;
	}
	
	private void handleSelectionChange(Object source){
		if(source.equals(getCapabilityViewer())){
			currentSelection = ((IStructuredSelection)getCapabilityViewer().getSelection()).getFirstElement();
			if(currentSelection instanceof Capability){
				addCapValueButton.setEnabled(true);
				displayComposite(getCapabilityDataGroup());
			}
			else if(currentSelection instanceof CapValue){
				addCapValueButton.setEnabled(false);
				displayComposite(getCapValueDataGroup());
			}
			else{
				addCapValueButton.setEnabled(false);
				displayComposite(getNothingToDisplayGroup());
			}
			getEditController().updateView(null);
		}
	}
	
	private TreeViewer getCapabilityViewer() {
		return capabilityViewer;
	}

	@Override
	public void okPressed() {
//		if(capNameText.isFocusControl() || capValueText.isFocusControl()){
			CapabilityDefinitionDialog.this.handleModifyEvent();
			shell.setFocus();
//		}else{
			close();
//		}
	}
	
	private EditController getEditController() {
		if(editController == null){
			editController = new EditController(){
				// Just delegate all update methods from the controller to
				// methods from the dialog.
				@Override
				protected void doUpdateView(Notification n) {
					CapabilityDefinitionDialog.this.doUpdateView(n);
				}

				@Override
				protected void enterPressed(Widget source) {
					applyCapValuesData();
					applyCapabilityData();
					CapabilityDefinitionDialog.this.handleModifyEvent();
				}

				@Override
				protected void focusOutEvent(Widget source) {
					applyCapValuesData();
					applyCapabilityData();
					CapabilityDefinitionDialog.this.handleModifyEvent();
				}

				@Override
				protected void modifyEvent(Widget source) {
					// TODO Auto-generated method stub
					
				}

				@Override
				protected void selectionEvent(Object source) {
					handleSelectionChange(source);
				}
				
			};
		}
		return editController;
	}
	
	private void doUpdateView(Notification n){
		if(currentSelection instanceof Capability){
			refreshCapabilityGroup();
		}else if(currentSelection instanceof CapValue){
			refreshCapValueGroup();
		}
		
		getCapabilityViewer().refresh(true);
	}
	
	private void handleModifyEvent(){
		if(getCapabilityDataGroup().isVisible())
		{
			applyCapabilityData();
		}
		else if (getCapValueDataGroup().isVisible())
		{
			applyCapValuesData();
		}
	}

	private void applyCapValuesData() {
		final CapValue capVal = (CapValue)currentSelection;
		domain.getCommandStack().execute(
				new RecordingCommand(domain){
					@Override
					protected void doExecute() {
						capVal.setName(capValueText.getText());
					}
				});
	}

	private void applyCapabilityData() {
		Capability cap = (Capability)currentSelection;		
		domain.getCommandStack().execute(
				SetCommand.create(
						domain, 
						cap, 
						AlicaPackage.eINSTANCE.getPlanElement_Name(), 
						capNameText.getText()));
	}
}

