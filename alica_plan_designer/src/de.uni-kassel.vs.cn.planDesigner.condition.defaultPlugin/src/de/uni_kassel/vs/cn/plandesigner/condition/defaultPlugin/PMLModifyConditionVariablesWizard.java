package de.uni_kassel.vs.cn.plandesigner.condition.defaultPlugin;

import java.util.List;

import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.wizard.Wizard;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.Variable;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;


public class PMLModifyConditionVariablesWizard extends Wizard {
	
	private PMLModifyConditionVariables page;
	
	private Condition condition;
	private AbstractPlan plan;
		
	private PMLTransactionalEditingDomain domain;
	
	public PMLModifyConditionVariablesWizard(Condition condition, AbstractPlan plan) {
		super();
		setWindowTitle("Configure condition " +condition.getName());
		
		this.condition = condition;
		this.plan = plan;
	}
	
	@Override
	public void addPages() {
		page = new PMLModifyConditionVariables(condition,plan);
		addPage(page);
	}
	
	@Override
	public boolean performFinish() {
		// Perform the update		
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
				// Get the set of plans which should now be in the planType
				List<Variable> varsToAdd = page.getConditionViewerList();
				
				// Remove all plans from he plantype 
				condition.getVars().clear();
				
				// Add all plans to the planType
				condition.getVars().addAll(varsToAdd);
								
			}

			public void redo() {
				// Nothing to redo since we cannot undo
			}
			
		});
		
		return true;
	}
	private PMLTransactionalEditingDomain getEditingDomain() {
		if(domain == null)
			domain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
					PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		
		return domain;
	}
		
}
