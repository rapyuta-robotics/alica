package de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages;

import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.List;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Button;
import org.eclipse.jface.viewers.ListViewer;
import org.eclipse.swt.widgets.Combo;

public class Test extends WizardPage {
	private Text text_fluent;
	private Text text_type;
	private Text text_variables;

	/**
	 * Create the wizard.
	 */
	public Test() {
		super("wizardPage");
		setTitle("Wizard Page title");
		setDescription("Wizard Page description");
	}

	/**
	 * Create contents of the wizard.
	 * @param parent
	 */
	public void createControl(Composite parent) {
		Composite container = new Composite(parent, SWT.NONE);
		

		setControl(container);
		
		Label lblDomainDescription = new Label(container, SWT.NONE);
		lblDomainDescription.setBounds(10, 10, 158, 17);
		lblDomainDescription.setText("Domain Description:");
		
		Label lblTypes = new Label(container, SWT.NONE);
		lblTypes.setBounds(391, 10, 70, 17);
		lblTypes.setText("Types:");
		
		ListViewer listViewer_DD = new ListViewer(container, SWT.BORDER | SWT.V_SCROLL);
		List list = listViewer_DD.getList();
		list.setBounds(10, 33, 270, 131);
		
		ListViewer listViewer_types = new ListViewer(container, SWT.BORDER | SWT.V_SCROLL);
		List list_1 = listViewer_types.getList();
		list_1.setBounds(391, 33, 187, 131);
		
		Label lblFluent = new Label(container, SWT.NONE);
		lblFluent.setBounds(10, 170, 57, 17);
		lblFluent.setText("Fluent:");
		
		text_fluent = new Text(container, SWT.BORDER);
		text_fluent.setBounds(10, 191, 142, 27);
		
		Button btnAdd_fluent = new Button(container, SWT.NONE);
		btnAdd_fluent.setBounds(158, 189, 57, 29);
		btnAdd_fluent.setText("Add");
		
		Button btnRemove = new Button(container, SWT.NONE);
		btnRemove.setBounds(221, 189, 70, 29);
		btnRemove.setText("Remove");
		
		Label lblType = new Label(container, SWT.NONE);
		lblType.setBounds(330, 170, 70, 17);
		lblType.setText("Type:");
		
		text_type = new Text(container, SWT.BORDER);
		text_type.setBounds(325, 193, 129, 27);
		
		Button btnAdd_type = new Button(container, SWT.NONE);
		btnAdd_type.setBounds(460, 191, 57, 29);
		btnAdd_type.setText("Add");
		
		Button btnRemove_1 = new Button(container, SWT.NONE);
		btnRemove_1.setBounds(518, 191, 70, 29);
		btnRemove_1.setText("Remove");
		
		text_variables = new Text(container, SWT.BORDER);
		text_variables.setBounds(10, 219, 142, 27);
		
		Combo combo = new Combo(container, SWT.NONE);
		combo.setBounds(286, 68, 99, 29);
		
		Label lblVariable = new Label(container, SWT.NONE);
		lblVariable.setText("Variable:");
		lblVariable.setBounds(286, 33, 70, 29);
		
		Button btnAddTypeToVariable = new Button(container, SWT.NONE);
		btnAddTypeToVariable.setBounds(286, 103, 99, 29);
		btnAddTypeToVariable.setText("<< Add type");
		
	
	}
}
