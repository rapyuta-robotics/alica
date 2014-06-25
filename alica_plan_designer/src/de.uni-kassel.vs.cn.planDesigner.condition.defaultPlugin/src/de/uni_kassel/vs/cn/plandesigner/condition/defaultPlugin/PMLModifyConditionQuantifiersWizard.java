package de.uni_kassel.vs.cn.plandesigner.condition.defaultPlugin;

import java.util.ArrayList;

import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.wizard.Wizard;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.Condition;
import de.uni_kassel.vs.cn.planDesigner.alica.ForallAgents;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.plandesigner.condition.defaultPlugin.PMLModifyConditionQuantifiers.GuiQuantifier;


public class PMLModifyConditionQuantifiersWizard extends Wizard {
	private PMLModifyConditionQuantifiers page;
	private Condition condition;
	private Plan plan;
	
	private PMLTransactionalEditingDomain domain;	
	
	public PMLModifyConditionQuantifiersWizard(Condition condition, Plan plan) {
		super();
		setWindowTitle("Configure condition " +condition.getName());
		
		this.condition = condition;
		this.plan = plan;
	}
	
	@Override
	public void addPages() {
		page = new PMLModifyConditionQuantifiers(condition,plan);
		addPage(page);
	}
	
	@Override
	public boolean performFinish() {
		// Perform the update
		ArrayList<GuiQuantifier> gqs = page.GetResult();
		for(GuiQuantifier gq:gqs) {
			if (gq.scope == null || gq.sorts.size()==0) return false;
		}
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
				
				ArrayList<GuiQuantifier> gqs = page.GetResult();
								
				condition.getQuantifiers().clear();
				for(GuiQuantifier gq: gqs) {
					if(gq.type.equals("ForallAgents")) {
						ForallAgents fa = AlicaFactory.eINSTANCE.createForallAgents();
						fa.setScope(gq.scope);
						for(String s : gq.sorts) {
							fa.getSorts().add(s);
						}
						condition.getQuantifiers().add(fa);
					}
				}
				
								
			}

			public void redo() {
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
