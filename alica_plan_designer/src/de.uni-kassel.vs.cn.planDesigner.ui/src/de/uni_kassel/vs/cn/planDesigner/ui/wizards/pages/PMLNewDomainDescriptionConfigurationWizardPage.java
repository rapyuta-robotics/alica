package de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages;

import java.util.ArrayList;

import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.ListViewer;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.MouseEvent;
import org.eclipse.swt.events.MouseListener;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Combo;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.List;
import org.eclipse.swt.widgets.Text;

import com.sun.org.apache.bcel.internal.generic.FDIV;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.Constant;
import de.uni_kassel.vs.cn.planDesigner.alica.DomainDescription;
import de.uni_kassel.vs.cn.planDesigner.alica.Fluent;
import de.uni_kassel.vs.cn.planDesigner.alica.FluentParameters;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;

public class PMLNewDomainDescriptionConfigurationWizardPage extends WizardPage {
	private PMLTransactionalEditingDomain domain;
	private DomainDescription dm;
	private Text text_fluent;
	private Text constant_def;
	private Text text_variables;
	private Text text_type; 

	public PMLNewDomainDescriptionConfigurationWizardPage(PMLTransactionalEditingDomain domain) {
		this(domain, null);
	}


	public PMLNewDomainDescriptionConfigurationWizardPage(PMLTransactionalEditingDomain domain, DomainDescription type) {
		super("New DomainDescription");
		setTitle("Configure the DomainDescription");
		this.dm = type;
		this.domain = domain;
	}

	@Override
	public void createControl(final Composite parent) {
		parent.setBounds(50, 100, 800, 800);
		final Composite container = new Composite(parent, SWT.NONE);

		setControl(container);

		Label lblDomainDescription = new Label(container, SWT.NONE);
		lblDomainDescription.setBounds(10, 10, 158, 17);
		lblDomainDescription.setText("Domain Description:");

		Label lblTypes = new Label(container, SWT.NONE);
		lblTypes.setBounds(391, 10, 70, 17);
		lblTypes.setText("Types:");

		ListViewer listViewer_DD = new ListViewer(container, SWT.BORDER | SWT.V_SCROLL | SWT.H_SCROLL);
		final List listDd = listViewer_DD.getList();
		listDd.setBounds(10, 33, 270, 131);

		ListViewer listViewer_types = new ListViewer(container, SWT.BORDER | SWT.V_SCROLL | SWT.H_SCROLL);
		final List listType = listViewer_types.getList();
		listType.setBounds(391, 33, 187, 131);

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

		final Combo combo = new Combo(container, SWT.NONE);
		combo.setBounds(286, 68, 99, 29);
		
		Label lblVariable = new Label(container, SWT.NONE);
		lblVariable.setText("Variable:");
		lblVariable.setBounds(286, 33, 70, 29);
		
		Button btnAddTypeToVariable = new Button(container, SWT.NONE);
		btnAddTypeToVariable.setBounds(286, 103, 99, 29);
		btnAddTypeToVariable.setText("<< Add type");

		for(Fluent f : dm.getFluents()){
			String text = f.getName() + "("; 
			for(FluentParameters p : f.getFormula()){
				if(p.getParameter().equals(f.getFormula().get(f.getFormula().size() - 1).getParameter())){
					text = text +  p.getParameter();
					if(p.getType() != null && !p.getType().equals("")){
						text = text + "[" + p.getType() + "]";
					}
				}else{
					text = text + p.getParameter();
					if(!p.getType().equals("")){
						text = text + "[" + p.getType() + "]" + ",";
					}else{
						text = text + ",";
					}
				}
			}

			listDd.add(text + ")");

		}
		for(String types : dm.getTypes()){
			listType.add(types);
		}
		btnAdd_type.addMouseListener(new MouseListener() {
			@Override
			public void mouseUp(MouseEvent e) {
			}
			@Override
			public void mouseDown(MouseEvent e) {
				if(!text_type.getText().equals("") && !checkOfDuplicate(text_type.getText(), listType) && Character.isUpperCase(text_type.getText().charAt(0))){
					listType.add(text_type.getText());
					domain.getCommandStack().execute(new RecordingCommand(domain) {
						@Override
						protected void doExecute() {
							dm.getTypes().add(text_type.getText());

						}
					});
					text_type.setText("");
				}else {
					MessageDialog.openWarning(parent.getShell(), "Error", "Type should starts with Uppercase,"
							+ "supposed to be not empty and the same should not ne twice"); 
				}
			}
			@Override
			public void mouseDoubleClick(MouseEvent e) {
			}
		});
		btnRemove_1.addMouseListener(new MouseListener() {
			@Override
			public void mouseUp(MouseEvent e) {
			}
			@Override
			public void mouseDown(MouseEvent e) {
				String[] listString = listType.getSelection();
				if(!(listString.length == 0)){
					for(String l : listString){
						listType.remove(l);
						final int i = dm.getTypes().indexOf(l);
						domain.getCommandStack().execute(new RecordingCommand(domain) {
							@Override
							protected void doExecute() {
								dm.getTypes().remove(i);

							}
						});
					}
				}
			}
			@Override
			public void mouseDoubleClick(MouseEvent e) {
			}
		});

		btnAdd_fluent.addMouseListener(new MouseListener() {
			@Override
			public void mouseUp(MouseEvent e) {
			}
			@Override
			public void mouseDown(MouseEvent e) {
				if(!text_fluent.getText().equals("") && !text_variables.getText().equals("") && Character.isUpperCase(text_fluent.getText().charAt(0))){
					final Fluent fluent = AlicaFactory.eINSTANCE.createFluent();
					fluent.setName(text_fluent.getText());
					String show = text_fluent.getText() + "(";
					if(text_variables.getText().contains(",")){
						String[] var = text_variables.getText().split(",");
						for(String v : var){
							final FluentParameters fluentPara = AlicaFactory.eINSTANCE.createFluentParameters();
							if(var[var.length -1].equals(v)){
								show = show + v;
								fluentPara.setParameter(v);
								fluent.getFormula().add(fluentPara);
							}else {
								show = show + v + ",";
								fluentPara.setParameter(v);
								fluent.getFormula().add(fluentPara);
							}
						}
						show = show +")";
					}else{
						final FluentParameters fluentPara = AlicaFactory.eINSTANCE.createFluentParameters();
						fluentPara.setParameter( text_variables.getText());
						fluentPara.setType("Robot");
						show = show + text_variables.getText() + ")";
						fluent.getFormula().add(fluentPara);
					}
					listDd.add(show);
					//Need a write premission!
					domain.getCommandStack().execute(new RecordingCommand(domain) {

						@Override
						protected void doExecute() {
							dm.getFluents().add(fluent);

						}
					});
					text_fluent.setText("");
					text_variables.setText("");

				}else {
					MessageDialog.openWarning(parent.getShell(), "Error", "Fluent should starts with Uppercase,"
							+ "supposed to be not empty, should not be twice"
							+ "and formulas the be correctly."); 
				}
			}

			@Override
			public void mouseDoubleClick(MouseEvent e) {

			}
		});

		btnRemove.addMouseListener(new MouseListener() {
			@Override
			public void mouseUp(MouseEvent e) {
			}
			@Override
			public void mouseDown(MouseEvent e) {
				int fluentIndex = listDd.getSelectionIndex();
				if( fluentIndex < 0 ){
					return;
				}
				String fluent = listDd.getItem(fluentIndex).substring(0, 
						listDd.getItem(fluentIndex).indexOf("("));
				
				if( fluent.length() > 0 ){
					ArrayList<Fluent> deleteList = new ArrayList<Fluent>();
					for(Fluent f : dm.getFluents()){
						
						if( f.getName().equals(fluent)){
							deleteList.add(f);
						}
					}
					
					for(int i=0; i<deleteList.size(); i++){
						final Fluent f = deleteList.get(i);
						domain.getCommandStack().execute(new RecordingCommand(domain) {
							@Override
							protected void doExecute() {
								dm.getFluents().remove(f);

							}
						});
					}
					listDd.remove(fluentIndex);
				}
			}
			@Override
			public void mouseDoubleClick(MouseEvent e) {
			}
		});
		
		listDd.addMouseListener(new MouseListener() {
			@Override
			public void mouseUp(MouseEvent e) {
			}

			@Override
			public void mouseDown(MouseEvent e) {
				String[] list = listDd.getSelection();
				String name = "";
				for(String l : list){
					for(Character c : l.toCharArray()){
						if(c.equals('(')){
							break;
						}else {
							name = name + c;
						}
					}
				}
				for(Fluent f : dm.getFluents()){
					if(f.getName().equals(name)){
						combo.removeAll();
						for(FluentParameters para : f.getFormula()){
							combo.add(para.getParameter());
						}

					}
				}

			}
			@Override
			public void mouseDoubleClick(MouseEvent e) {
			}
		});

		btnAddTypeToVariable.addMouseListener(new MouseListener() {
			@Override
			public void mouseUp(MouseEvent e) {
			}

			@Override
			public void mouseDown(MouseEvent e) {
				int index = listType.getSelectionIndex();
				int fluentIndex = listDd.getSelectionIndex();
				if( index < 0 || fluentIndex < 0 ){
					return;
				}
				final String type = listType.getItem(index);
				String parameter = combo.getText();
				String totalFluent = listDd.getItem(fluentIndex);
				String fluent = listDd.getItem(fluentIndex).substring(0, 
						listDd.getItem(fluentIndex).indexOf("("));
				
				if( totalFluent.contains(parameter) ){
					
					String newFluent = fluent + "(";
					Fluent tmpFluent = null;
					for(Fluent f : dm.getFluents()){
						if( f.getName().equals(fluent)){
							tmpFluent = f;
							break;
						}
					}
					
					if( tmpFluent == null ){
						return;
					}
					
					int paraSize = tmpFluent.getFormula().size();
					int i=0;
					for(final FluentParameters para : tmpFluent.getFormula()){
						
						newFluent += para.getParameter();
								
						if( tmpFluent.getName().equals(fluent) && para.getParameter().equals(parameter)  ){
							
							newFluent += " [" + type + "]";
							
							domain.getCommandStack().execute(new RecordingCommand(domain) {
								@Override
								protected void doExecute() {
									para.setType(type);

								}
							});
						} else if( para.getType() != null && para.getType().length() > 0 ){
							newFluent += " [" + para.getType() + "]";
						}
						
						if( (i+1) < paraSize ){
							newFluent += ",";
						}
							
						i++;
					}
					newFluent += ")";
					listDd.setItem(fluentIndex, newFluent);
				}
			}
			@Override
			public void mouseDoubleClick(MouseEvent e) {
			}
		});
		
		Label constants = new Label(container, SWT.NONE);
		constants.setBounds(10, 290, 158, 17);
		constants.setText("Constants: ");
		
		ListViewer listViewerConstants = new ListViewer(container, SWT.BORDER | SWT.V_SCROLL | SWT.H_SCROLL);
		final List listConstants = listViewerConstants.getList();
		listConstants.setBounds(10, 315, 270, 131);
		
		for(Constant c : dm.getConstants()){
			listConstants.add(c.getName() + " [" + c.getType() + "]");
		}
		
		Label nameConstant = new Label(container, SWT.NONE);
		nameConstant.setBounds(10, 465, 158, 17);
		nameConstant.setText("Name: ");
		
		constant_def = new Text(container, SWT.BORDER);
		constant_def.setBounds(10, 485, 142, 27);
		
		Button constant_add = new Button(container, SWT.NONE);
		constant_add.setBounds(158, 483, 57, 29);
		constant_add.setText("Add");
		
		Button constant_remove = new Button(container, SWT.NONE);
		constant_remove.setBounds(228, 483, 70, 29);
		constant_remove.setText("Remove");
		
		//button listener
		constant_add.addMouseListener(new MouseListener() {
			@Override
			public void mouseUp(MouseEvent e) {
			}
			@Override
			public void mouseDown(MouseEvent e) {
				int indexOfType = listType.getSelectionIndex();
				if( indexOfType < 0){
					return;
				}
				final String type = listType.getItem(indexOfType);
				String constantName = constant_def.getText();
				if( constantName != null &&  constantName.length() > 0 && type != null && type.length() > 0 ){
					
					String show = constant_def.getText() + " [" + type + "]";
					
					listConstants.add(show);
					//Need a write premission!
					domain.getCommandStack().execute(new RecordingCommand(domain) {

						@Override
						protected void doExecute() {
							final Constant constant = AlicaFactory.eINSTANCE.createConstant();
							constant.setName(constant_def.getText());
							constant.setType(type);
							dm.getConstants().add(constant);

						}
					});
					constant_def.setText("");
					

				}else {
					MessageDialog.openWarning(parent.getShell(), "Error", "No type selcted,"
							+ " or no constant entered!"); 
				}
			}

			@Override
			public void mouseDoubleClick(MouseEvent e) {

			}
		});
		
		constant_remove.addMouseListener(new MouseListener() {
			@Override
			public void mouseUp(MouseEvent e) {
			}
			@Override
			public void mouseDown(MouseEvent e) {
				int index = listConstants.getSelectionIndex();
				if ( index < 0 ){
					return;
				}
				String cNameShow = listConstants.getItem(index);
				String cName = cNameShow.substring(0, cNameShow.indexOf("[")-1);
				String type = cNameShow.substring(cNameShow.indexOf("[")+1, cNameShow.indexOf("]")-1);
				listConstants.remove(index);
				
				for(final Constant c : dm.getConstants()){
					
					if( c.getName().equals(cName) && c.getType().equals(type)) {
						domain.getCommandStack().execute(new RecordingCommand(domain) {
							@Override
							protected void doExecute() {
								dm.getTypes().remove(c);
	
							}
						});
					}
				}
			}
			@Override
			public void mouseDoubleClick(MouseEvent e) {
			}
		});
		
		this.getWizard().getContainer().getShell().setMinimumSize(800, 800);
	}

	private boolean checkOfDuplicate(String string, List listType) {
		for(String s : listType.getItems()){
			if(s.equals(string)){
				return true;
			}
		}

		return false;
	}
}
