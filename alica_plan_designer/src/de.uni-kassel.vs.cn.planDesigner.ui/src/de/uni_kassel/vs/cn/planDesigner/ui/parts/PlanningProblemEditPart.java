package de.uni_kassel.vs.cn.planDesigner.ui.parts;

import org.eclipse.draw2d.Label;
import org.eclipse.emf.common.notify.Notification;
import org.eclipse.gef.DragTracker;
import org.eclipse.gef.Request;
import org.eclipse.gef.RequestConstants;
import org.eclipse.gef.tools.DragEditPartsTracker;
import org.eclipse.jface.wizard.WizardDialog;

import de.uni_kassel.vs.cn.planDesigner.alica.PlanningProblem;
import de.uni_kassel.vs.cn.planDesigner.ui.PlanDesignerActivator;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLPlanningProblemConfigurationWizard;

public class PlanningProblemEditPart extends AbstractPlanStateEditPart {
	
	@Override
	protected Label createNameLabel() {
		Label l = new Label(getPlanElement().getName());
		
		PlanDesignerActivator plugin = PlanDesignerActivator.getDefault();
		
		l.setIcon(plugin.getImageRegistry().get(PlanDesignerConstants.ICON_PLANNING_PROBLEM_16));
		
		return l;
	}
	
	@Override
	protected void handleModelChanged(Notification n) {
		super.handleModelChanged(n);
	}
	
	@Override
	public DragTracker getDragTracker(Request request)
	{
		return new DragEditPartsTracker(this);
	}
	
	@Override
	public void performRequest(Request req)
	{
		if(req.getType() == RequestConstants.REQ_OPEN)
		{
			// Create a plantype configuration wizard and initialize it with the plantype
			PMLPlanningProblemConfigurationWizard wiz = new PMLPlanningProblemConfigurationWizard((PlanningProblem)getModel());
			
			WizardDialog dialog = new WizardDialog(getViewer().getControl().getShell(),wiz);
			
			
			dialog.setBlockOnOpen(true);
			dialog.open();
		}
		else
		{
			super.performRequest(req);
		}
	}
}
