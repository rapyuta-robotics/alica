package de.uni_kassel.vs.cn.planDesigner.ui.actions;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Set;

import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.Path;
import org.eclipse.emf.common.command.CompoundCommand;
import org.eclipse.emf.ecore.EStructuralFeature.Setting;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.edit.command.RemoveCommand;
import org.eclipse.emf.transaction.RecordingCommand;
import org.eclipse.emf.transaction.RunnableWithResult;
import org.eclipse.emf.transaction.TransactionalEditingDomain;
import org.eclipse.jface.action.Action;
import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.jface.viewers.ISelection;
import org.eclipse.jface.viewers.IStructuredSelection;
import org.eclipse.jface.viewers.StructuredViewer;
import org.eclipse.jface.window.Window;
import org.eclipse.swt.widgets.Shell;

import de.uni_kassel.vs.cn.planDesigner.alica.AlicaPackage;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.RemoveReadOnlyCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanUsageDialog;

public class DeletePlanFromRepositoryAction extends Action {
	private StructuredViewer viewer;
	private Collection<Setting> usages;
	private PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
			PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
	private ArrayList<Plan> plans = new ArrayList<Plan>();
	public DeletePlanFromRepositoryAction(StructuredViewer viewer){
		this.viewer = viewer;
	}

	@Override
	public void run() {
		ISelection selection = viewer.getSelection();
		if(selection != null && shouldReallyDelete()){
			Plan planToDelete = (Plan)((IStructuredSelection)selection).getFirstElement();
			usages = getUsages(planToDelete);
			List<State> state = new ArrayList<State>();
			plans = new ArrayList<Plan>();
			if(!usages.isEmpty()){
				for(Setting setting : usages){
					if(setting.getEObject() instanceof State){
						state.add((State)setting.getEObject());
						if(!plans.contains(((State)setting.getEObject()))){
							plans.add(((State)setting.getEObject()).getInPlan());
						}
					}
				}
			}
			CompoundCommand cmp = new CompoundCommand();
			if(plans.size() > 0 && shouldProceed()){
				for(State s : state){
					cmp.append(RemoveCommand.create(editingDomain, s, AlicaPackage.eINSTANCE.getState_Plans(), planToDelete));
				}
				for(final Plan p : plans){
					cmp.append(new RecordingCommand(editingDomain) {
						
						@Override
						protected void doExecute() {
							try {
								p.eResource().save(AlicaSerializationHelper.getInstance().getLoadSaveOptions());
							} catch (IOException e) {
								e.printStackTrace();
							}
						}
					});
				}
				// Enable write permission
				editingDomain.getCommandStack().execute(new RemoveReadOnlyCommand(editingDomain));
				final IFile resourceFile = ResourcesPlugin.getWorkspace().getRoot().getFile(new Path(planToDelete.eResource().getURI().toPlatformString(true)));
				PlanEditorUtils.getRemovedResourcesUtil().add(planToDelete.eResource());
				editingDomain.getCommandStack().execute(cmp.unwrap());
				try {
					resourceFile.delete(true, null);
				} catch (CoreException e) {
					e.printStackTrace();
				}
			}
		}
	}

	private boolean shouldProceed() {
		final Shell shell = viewer.getControl().getShell();
		RunnableWithResult<Boolean> run = new RunnableWithResult.Impl<Boolean>(){
			public void run() {
				PlanUsageDialog dia = new PlanUsageDialog(shell, plans);
				if(dia.open() == Window.OK)
					setResult(true);
				else
					setResult(false);
			}
		};

		shell.getDisplay().syncExec(run);
		return run.getResult();

	}

	private Collection<Setting> getUsages(Plan planToDelete) {
		if(usages == null){
			// Load all plans which resist in the workspace.
			Set<IFile> planFiles = PlanEditorUtils.collectAllFilesWithExtension("pml");
			for(IFile file : planFiles){
				editingDomain.load(file);
			}

			usages = EcoreUtil.UsageCrossReferencer.find(planToDelete, editingDomain.getResourceSet());
		}
		return usages;
	}
	private boolean shouldReallyDelete() {
		final Shell shell = viewer.getControl().getShell();
		RunnableWithResult<Boolean> run = new RunnableWithResult.Impl<Boolean>(){
			public void run() {
				if(MessageDialog.openQuestion(
						shell, 
						"Confirm delete", 
						"Do you really want to delete the selected plan?"))
					setResult(true);
				else
					setResult(false);
			}
		};

		shell.getDisplay().syncExec(run);
		return run.getResult();
	}

}
