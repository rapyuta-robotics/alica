package de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages;

import java.awt.Window;
import java.awt.event.MouseEvent;
import java.util.ArrayList;

import javax.swing.JOptionPane;

import org.eclipse.swt.widgets.Composite;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.Planner;
import de.uni_kassel.vs.cn.planDesigner.alica.Planners;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;

import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.LabelProvider;
import org.eclipse.jface.wizard.WizardPage;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.MouseListener;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.List;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;

public class PMLNewPlannerConfigurationWizardPage extends WizardPage{
	private Text text;
	private Planners pp;
	private PMLTransactionalEditingDomain domain;
	private int counter = 0;
	List list;
	ArrayList<Planner> planner = new ArrayList<>();
	private Text txt_command;

	public ArrayList<Planner> getPlanner() {
		return planner;
	}


	/**
	 * @wbp.parser.constructor
	 */
	public PMLNewPlannerConfigurationWizardPage(PMLTransactionalEditingDomain domain) {
		this(domain, null);
	}
	
	
	public PMLNewPlannerConfigurationWizardPage(PMLTransactionalEditingDomain domain, Planners type) {
		super("New Planner");
		setTitle("Configure the Planner");
		this.pp = type;
		this.domain = domain;
	}

	@Override
	public void createControl(final Composite parent) {
		
		// TODO Auto-generated method stub
		Composite container = new Composite(parent, SWT.NONE);
		

		setControl(container);
		
		Label lblPlanner = new Label(container, SWT.NONE);
		lblPlanner.setBounds(61, 10, 123, 17);
		lblPlanner.setText("Planner");
		
		list = new List(container, SWT.BORDER);
		list.setBounds(61, 33, 370, 123);
		
		text = new Text(container, SWT.BORDER);
		text.setBounds(61, 185, 168, 27);
		
		Button btnAdd = new Button(container, SWT.NONE);
		btnAdd.setBounds(243, 183, 91, 29);
		btnAdd.setText("Add");
		
		Button btnRemove = new Button(container, SWT.NONE);
		btnRemove.setBounds(340, 183, 91, 29);
		btnRemove.setText("Remove");
		
		Label label = new Label(container, SWT.SEPARATOR | SWT.VERTICAL);
		label.setBounds(458, 10, 2, 231);
		
		final Label lblType = new Label(container, SWT.NONE);
		lblType.setBounds(466, 33, 47, 17);
		lblType.setText("Type:");
		
		final Label label_1 = new Label(container, SWT.NONE);
		label_1.setBounds(512, 33, 55, 17);
		label_1.setText("" + counter);
		
		Label lblPlannerName = new Label(container, SWT.NONE);
		lblPlannerName.setBounds(61, 162, 123, 17);
		lblPlannerName.setText("Planner Name:");
		
		Label lblCommand = new Label(container, SWT.NONE);
		lblCommand.setBounds(465, 66, 80, 17);
		lblCommand.setText("Command:");
		
		final Label lbl_command = new Label(container, SWT.NONE);
		lbl_command.setBounds(475, 91, 105, 17);
		
		txt_command = new Text(container, SWT.BORDER);
		txt_command.setBounds(61, 241, 168, 27);
		
		Label lblCommand_1 = new Label(container, SWT.NONE);
		lblCommand_1.setBounds(61, 218, 80, 17);
		lblCommand_1.setText("Command:");
		
		Button btn_modify = new Button(container, SWT.NONE);
		btn_modify.setBounds(243, 212, 188, 29);
		btn_modify.setText("Modify Name");
		
		Button btn_modifyCommand = new Button(container, SWT.NONE);
		btn_modifyCommand.addSelectionListener(new SelectionAdapter() {
			@Override
			public void widgetSelected(SelectionEvent e) {
			}
		});
		btn_modifyCommand.setBounds(243, 241, 188, 29);
		btn_modifyCommand.setText("Modify Command");
		
		for(Planner p : pp.getPlanners()){
			if(p != null){
				list.add(p.getName());
				planner.add(p);
				if(p.getType() > counter){
					counter = p.getType() + 1;
				}
			}
		}
		btn_modifyCommand.addMouseListener(new MouseListener() {
			@Override
			public void mouseUp(org.eclipse.swt.events.MouseEvent e) {
			}
			
			@Override
			public void mouseDown(org.eclipse.swt.events.MouseEvent e) {
				
				if(list.getSelectionIndex() != -1){
					String[] listString = list.getSelection();
					
					if(!(listString.length == 0)){
						for(final Planner p : planner){
							if(p.getName().equals(listString[0])){
								final ChangeCommandOrNameDialog dialog;
								if(p.getCommand() == null){
									dialog = new ChangeCommandOrNameDialog(parent.getShell(), "Hatte kein Command");
								}else{
									dialog = new ChangeCommandOrNameDialog(parent.getShell(), p.getCommand());
								}
								
								if(dialog.open() == org.eclipse.jface.window.Window.OK){
									domain.getCommandStack().execute(new RecordingCommand(domain){
										@Override
										protected void doExecute() {
											p.setCommand(dialog.getStr());
										}
									});
								}
							}
						}
					}
				}
			}
			@Override
			public void mouseDoubleClick(org.eclipse.swt.events.MouseEvent e) {
			}
		});
		btn_modify.addMouseListener(new MouseListener() {
			@Override
			public void mouseUp(org.eclipse.swt.events.MouseEvent e) {
			}
			
			@Override
			public void mouseDown(org.eclipse.swt.events.MouseEvent e) {
				
				if(list.getSelectionIndex() != -1){
					String[] listString = list.getSelection();
					
					if(!(listString.length == 0)){
						for(final Planner p : planner){
							if(p.getName().equals(listString[0])){
								final ChangeCommandOrNameDialog dialog;
								if(p.getName() == null){
									dialog = new ChangeCommandOrNameDialog(parent.getShell(), "Hatte keinen Namen");
								}else{
									dialog = new ChangeCommandOrNameDialog(parent.getShell(), p.getName());
								}
								
								if(dialog.open() == org.eclipse.jface.window.Window.OK){
									domain.getCommandStack().execute(new RecordingCommand(domain){
										@Override
										protected void doExecute() {
											p.setName(dialog.getStr());
										}
									});
								}
							}
						}
					}
				}
			}
			@Override
			public void mouseDoubleClick(org.eclipse.swt.events.MouseEvent e) {
			}
		});
		
		btnAdd.addMouseListener(new MouseListener(){
			@Override
			public void mouseDoubleClick(org.eclipse.swt.events.MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
			@Override
			public void mouseDown(org.eclipse.swt.events.MouseEvent e) {
				// TODO Auto-generated method stub
				if(!text.getText().equals("") && Character.isUpperCase(text.getText().charAt(0)) && duplicate(text.getText())){
					String txtString = text.getText();
					String command = txt_command.getText();
					text.setText("");
					txt_command.setText("");
					final Planner p = AlicaFactory.eINSTANCE.createPlanner();
					p.setName(txtString);
					p.setCommand(command);
					counter++;
					p.setType(counter);
					list.add(txtString);
					
					domain.getCommandStack().execute(new RecordingCommand(domain){
						@Override
						protected void doExecute() {
							planner.add(p);
						}
					});
						
				}else {
					MessageDialog.openWarning(parent.getShell(), "Error", "Planner should starts with Uppercase,"
							+ "supposed to be not empty and the same should not ne twice"); 
				}
			}
			@Override
			public void mouseUp(org.eclipse.swt.events.MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
		});
		
		btnRemove.addMouseListener(new MouseListener(){
			@Override
			public void mouseDoubleClick(org.eclipse.swt.events.MouseEvent e) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void mouseDown(org.eclipse.swt.events.MouseEvent e) {
				String[] listString = list.getSelection();
				if(!(listString.length == 0)){
					for(String l : listString){
						list.remove(l);
						planner.remove(removeThat(l));
					}
				}
			}
			@Override
			public void mouseUp(org.eclipse.swt.events.MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
			
		});
		
		list.addMouseListener(new MouseListener() {
			
			@Override
			public void mouseUp(org.eclipse.swt.events.MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void mouseDown(org.eclipse.swt.events.MouseEvent e) {
				// TODO Auto-generated method stub
				if(list.getSelectionIndex() != -1){
					String[] listString = list.getSelection();
					if(!(listString.length == 0)){
						for(Planner p : planner){
							if(p.getName().equals(listString[0])){
								label_1.setText("" + p.getType());
								lbl_command.setText("" + p.getCommand());
							}
						}
					}
				}
			}
			@Override
			public void mouseDoubleClick(org.eclipse.swt.events.MouseEvent e) {
				// TODO Auto-generated method stub
				
			}
		});
		
	}
	public boolean duplicate(String name){
		for(Planner p : planner){
			if(p.getName().equals(name)){
				return false;
			}
		}
		return true;
	}
	public Planner removeThat (String name){
		for(Planner p : planner){
			if(p.getName().equals(name)){
				return p;
			}
		}
		return null;
	}
}
