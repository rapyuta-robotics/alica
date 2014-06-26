package de.uni_kassel.vs.cn.planDesigner.ui.editors;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IWorkspace;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.IPath;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.wizard.WizardDialog;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.ui.IEditorLauncher;

import de.uni_kassel.vs.cn.planDesigner.alica.Planners;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.wizards.PMLNewPlannerConfigurationWizard;

public class PlannerLauncher implements IEditorLauncher{
	private PMLTransactionalEditingDomain editingDomain;
	@Override
	public void open(IPath file) {
		// TODO Auto-generated method stub
		// TODO Auto-generated method stub
		Shell shell = Display.getDefault().getActiveShell();
		IWorkspace workspace= ResourcesPlugin.getWorkspace();    
		IFile ifile= workspace.getRoot().getFileForLocation(file);
		Resource res = getEditingDomain().load(ifile);
			
		Planners type = (Planners) res.getContents().get(0);
		// Create a plantype configuration wizard and initialize it with the plantype
		PMLNewPlannerConfigurationWizard wiz = new PMLNewPlannerConfigurationWizard(type);
		WizardDialog dialog = new WizardDialog(shell, wiz);
			dialog.setBlockOnOpen(true);
			dialog.open();	
	}
			
	public PMLTransactionalEditingDomain getEditingDomain() {
		if(editingDomain == null){
		// Establish a connecton to the editingDomain
			editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
			PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
		}
				return editingDomain;
	}
}

