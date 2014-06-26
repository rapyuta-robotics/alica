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
import org.eclipse.emf.common.util.EList;
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
import de.uni_kassel.vs.cn.planDesigner.alica.Behaviour;
import de.uni_kassel.vs.cn.planDesigner.alica.BehaviourConfiguration;
import de.uni_kassel.vs.cn.planDesigner.alica.Plan;
import de.uni_kassel.vs.cn.planDesigner.alica.State;
import de.uni_kassel.vs.cn.planDesigner.alica.util.AlicaSerializationHelper;
import de.uni_kassel.vs.cn.planDesigner.ui.commands.RemoveReadOnlyCommand;
import de.uni_kassel.vs.cn.planDesigner.ui.edit.PMLTransactionalEditingDomain;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanDesignerConstants;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanEditorUtils;
import de.uni_kassel.vs.cn.planDesigner.ui.util.PlanUsageDialog;

public class DeleteBehaviourConfigurationAction extends Action {
	private StructuredViewer viewer;
	private Collection<Setting> usagesBeh;
	private PMLTransactionalEditingDomain editingDomain = (PMLTransactionalEditingDomain)TransactionalEditingDomain.Registry.INSTANCE.getEditingDomain(
			PlanDesignerConstants.PML_TRANSACTIONAL_EDITING_DOMAIN_ID);
	private ArrayList<Plan> plans = new ArrayList<Plan>();

	public DeleteBehaviourConfigurationAction(StructuredViewer viewer){
		this.viewer = viewer;
	}
	private class Pair{
		Object state, behConf;
		public Object getState() {
			return state;
		}
		public Object getBehConf() {
			return behConf;
		}
		public Pair(Object a, Object b){
			this.state = a;
			this.behConf = b;
		}
	}
	@Override
	public void run() {
		ISelection selection = viewer.getSelection();
		if(selection != null && shouldReallyDelete()){
			//delete Behavior
			List<Pair> stateWithPair = new ArrayList<Pair>();
			if(((IStructuredSelection)selection).getFirstElement() instanceof Behaviour){
				Behaviour beh = (Behaviour)((IStructuredSelection)selection).getFirstElement();
				usagesBeh = getUsagesForConf(beh.getConfigurations());
				String path = beh.eResource().getURI().toPlatformString(true);
				if(!usagesBeh.isEmpty()){
					for(Setting setting : usagesBeh){
						if(setting.getEObject() instanceof State){
							State s = (State)setting.getEObject();
							for(BehaviourConfiguration b : beh.getConfigurations()){
								if(s.getPlans().contains(b)){
									stateWithPair.add(new Pair(s,b));
								}
							}
							if(!plans.contains(((State)setting.getEObject()))){
								plans.add(((State)setting.getEObject()).getInPlan());
							}
						}
					}
				}
				CompoundCommand cmp = new CompoundCommand();
				if(plans.size() > 0 && shouldProceed()){
					for(Pair s : stateWithPair){
						cmp.append(RemoveCommand.create(editingDomain, s.getState(), AlicaPackage.eINSTANCE.getState_Plans(), s.getBehConf()));
					}
					//REMOVE BEHCONF FROM BEH
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
					editingDomain.getCommandStack().execute(cmp.unwrap());
					final IFile resourceFile = ResourcesPlugin.getWorkspace().getRoot().getFile(new Path(path));
					try {
						resourceFile.delete(true, null);
					} catch (CoreException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					plans.clear();
					usagesBeh.clear();
				}
			}
			//delete BehaviourConf 
			else {
				List<State> state = new ArrayList<State>();
				boolean needToDelteTheFile = false;
				BehaviourConfiguration behaviourConf = (BehaviourConfiguration)((IStructuredSelection)selection).getFirstElement();
				usagesBeh = getUsages(behaviourConf);
				String path = behaviourConf.getBehaviour().eResource().getURI().toPlatformString(true);
				if(behaviourConf.getBehaviour().getConfigurations().size() <= 1){
					needToDelteTheFile = true;
				}
				if(!usagesBeh.isEmpty()){
					for(Setting setting : usagesBeh){
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
						cmp.append(RemoveCommand.create(editingDomain, s, AlicaPackage.eINSTANCE.getState_Plans(), behaviourConf));
					}
					//REMOVE BEHCONF FROM BEH
					cmp.append(RemoveCommand.create(editingDomain, behaviourConf.getBehaviour(), AlicaPackage.eINSTANCE.getBehaviour_Configurations(), behaviourConf));
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
					editingDomain.getCommandStack().execute(cmp.unwrap());
					//Delete the .beh file cause only 1 configuration was in this behaviour
					if(needToDelteTheFile){
						final IFile resourceFile = ResourcesPlugin.getWorkspace().getRoot().getFile(new Path(path));
						try {
							resourceFile.delete(true, null);
						} catch (CoreException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
					plans.clear();
					usagesBeh.clear();
				}
			}
		}
	}

	private Collection<Setting> getUsagesForConf(EList<BehaviourConfiguration> configurations) {
		if(usagesBeh == null){
			// Load all plans which resist in the workspace.
			for(BehaviourConfiguration conf : configurations){	
				Set<IFile> planFiles = PlanEditorUtils.collectAllFilesWithExtension("pml");
				for(IFile file : planFiles){
					editingDomain.load(file);
				}
				usagesBeh = EcoreUtil.UsageCrossReferencer.find(conf, editingDomain.getResourceSet());
			}
		}
		return usagesBeh;
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

	private Collection<Setting> getUsages(BehaviourConfiguration planToDelete) {
		if(usagesBeh == null){
			// Load all plans which resist in the workspace.
			Set<IFile> planFiles = PlanEditorUtils.collectAllFilesWithExtension("pml");
			for(IFile file : planFiles){
				editingDomain.load(file);
			}
			usagesBeh = EcoreUtil.UsageCrossReferencer.find(planToDelete, editingDomain.getResourceSet());
		}
		return usagesBeh;
	}

	private boolean shouldReallyDelete() {
		final Shell shell = viewer.getControl().getShell();
		RunnableWithResult<Boolean> run = new RunnableWithResult.Impl<Boolean>(){
			public void run() {
				if(MessageDialog.openQuestion(
						shell, 
						"Confirm delete", 
						"Do you really want to delete the selected behaviour?"))
					setResult(true);
				else
					setResult(false);
			}
		};

		shell.getDisplay().syncExec(run);
		return run.getResult();
	}

}
