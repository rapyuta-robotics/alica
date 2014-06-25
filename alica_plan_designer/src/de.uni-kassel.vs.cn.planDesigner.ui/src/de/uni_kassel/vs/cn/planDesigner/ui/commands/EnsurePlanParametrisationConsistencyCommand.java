
package de.uni_kassel.vs.cn.planDesigner.ui.commands;

import org.eclipse.emf.common.command.AbstractCommand;

import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.PlanType;

public class EnsurePlanParametrisationConsistencyCommand extends AbstractCommand {
	private Plan plan;
	private PlanType pt;
	
	public EnsurePlanParametrisationConsistencyCommand(Plan plan){
		this.plan = plan;
	}
	public EnsurePlanParametrisationConsistencyCommand(PlanType pt){
		this.pt = pt;
	}
	public void execute() {
		if (plan!=null) plan.ensureParametrisationConsistency();
		else if (pt !=null) pt.ensureParametrisationConsistency();
	}

	public void redo() {
		// Nothing to do, since this command cannot be undone
	}

	@Override
	public boolean canUndo() {
		return false;
	}
	
	@Override
	public boolean canExecute() {
		return true;
	}

}
