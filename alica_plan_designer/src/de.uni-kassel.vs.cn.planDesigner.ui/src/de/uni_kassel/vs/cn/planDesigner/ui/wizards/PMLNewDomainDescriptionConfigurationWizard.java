package de.uni_kassel.vs.cn.planDesigner.ui.wizards;

import java.io.IOException;

import org.eclipse.emf.common.command.AbstractCommand;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.wizard.Wizard;
import org.eclipse.swt.widgets.List;

import de.uni_kassel.vs.cn.planDesigner.alica.AbstractPlan;
import de.uni_kassel.vs.cn.planDesigner.alica.AlicaFactory;
import de.uni_kassel.vs.cn.planDesigner.alica.DomainDescription;
import de.uni_kassel.vs.cn.planDesigner.alica.Planner;
import de.uni_kassel.vs.cn.planDesigner.alica.Planners;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;
import de.uni_kassel.vs.cn.planDesigner.alica.PostCondition;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.editors.DomainDescriptionLauncher;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewDomainDescriptionConfigurationWizardPage;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewPlannerConfigurationWizardPage;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.pages.PMLNewPlanningProblemConfigurationWizardPage;

public class PMLNewDomainDescriptionConfigurationWizard extends Wizard{

	private PMLNewDomainDescriptionConfigurationWizardPage page;
	
	private DomainDescription pp;
	
	private PMLTransactionalEditingDomain domain;
	
	public PMLNewDomainDescriptionConfigurationWizard(DomainDescription type) {
		super();
		setWindowTitle("Configure planningProblem " +type.getName());
		
		this.pp = type;
	}
	
	@Override
	public void addPages() {
		page = new PMLNewDomainDescriptionConfigurationWizardPage(getEditingDomain(), pp);
		addPage(page);
	}
	
	@Override
	public boolean performFinish() {
		// Save the resource where the plantype we are configuring is in
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
				// Save the planType
				Resource ppResource = pp.eResource();
				try {
					ppResource.save(ppResource.getResourceSet().getLoadOptions());
				} catch (IOException e) {
					e.printStackTrace();
				}
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
